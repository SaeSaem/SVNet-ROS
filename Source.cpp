/*
output 이 무엇인지, calibration 결과로
pose 와 팬, 틸트, 롤이 나오는 것.
카메라!

cameraMatrix prameter 6개
+왜곡계수 4개

4이상의 대응쌍(2D, 3D)

1. Gopro
2. Webcam

*/


#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>

using namespace std;
using namespace cv;


std::vector<cv::Point2f> Generate2DPoints();
std::vector<cv::Point3f> Generate3DPoints();

int main(int argc, char* argv[])
{
	// Read points
	std::vector<cv::Point2f> imagePoints = Generate2DPoints();
	std::vector<cv::Point3f> objectPoints = Generate3DPoints();

	std::cout << "There are " << imagePoints.size() << " imagePoints and " << objectPoints.size() << " objectPoints." << std::endl;
	cv::Mat cameraMatrix(3, 3, cv::DataType<double>::type);
	//cv::setIdentity(cameraMatrix);

	//std::cout << "Initial cameraMatrix: " << cameraMatrix << std::endl;

	//Gopro parameter
	cameraMatrix.at<double>(0, 0) = 1250.978;   // fx
	cameraMatrix.at<double>(0, 1) = 0;
	cameraMatrix.at<double>(0, 2) = 626.673;    // cx

	cameraMatrix.at<double>(1, 0) = 0;
	cameraMatrix.at<double>(1, 1) = 1257.900;    // fy
	cameraMatrix.at<double>(1, 2) = 354.239;     // cy

	cameraMatrix.at<double>(2, 0) = 0;
	cameraMatrix.at<double>(2, 1) = 0;
	cameraMatrix.at<double>(2, 2) = 1;

	cout << "cameraMatrix" << endl;
	cout << cameraMatrix << endl;

	cv::Mat distCoeffs(4, 1, cv::DataType<double>::type);
	distCoeffs.at<double>(0) = -0.318184;
	distCoeffs.at<double>(1) = 1.125858;
	distCoeffs.at<double>(2) = 0.004153;
	distCoeffs.at<double>(3) = -0.005453;

	cv::Mat rvec(3, 1, cv::DataType<double>::type);
	cv::Mat tvec(3, 1, cv::DataType<double>::type);

	cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

	cout << endl;

	std::cout << "rvec: " << rvec << std::endl;
	std::cout << "tvec: " << tvec << std::endl;

	cout << endl;

	Mat R;
	Rodrigues(rvec, R);
	Mat R_inv = R.inv();

	// camera position (X,Y,Z)
	Mat Cam_pos = -R_inv*tvec;
	double* p = (double*)Cam_pos.data;
	double X = p[0];
	double Y = p[1];
	double Z = p[2];

	cout << "camera position" << endl;
	cout << "X: " << X << endl;
	cout << "Y: " <<Y << endl;
	cout << "Z: "<< Z << endl;

	cout << endl;


	// 3D to 2D
	// zaxis 는 카메라의 광축 방향으로의 거리
	// 
	X = 1.5; // 중심점과 평행
	Y = 1.5; // 중심점과 평행
	double zaxis = 60;  // 광학축과 수직거리, 3D 데이터에서 가져오면 됩니다.
	cout << "3D to 2D" << endl;
	double x = cameraMatrix.at<double>(0, 0)*X / zaxis + cameraMatrix.at<double>(0, 2);
	double y = cameraMatrix.at<double>(1, 1)*Y / zaxis + cameraMatrix.at<double>(1, 2);
	cout << x << endl;
	cout << y << endl;
	cout << endl;

	// pan & titl
	double unit_z[] = { 0, 0, 1 };
	Mat Zc(3, 1, CV_64FC1, unit_z);
	Mat Zw = R_inv*Zc; // world coordinate of optical axis
	double* zw = (double*)Zw.data;

	double pan = atan2(zw[1], zw[0]) - CV_PI / 2;
	double tilt = atan2(zw[2], sqrt(zw[0] * zw[0] + zw[1] * zw[1]));

	// roll
	double unit_x[] = { 1, 0 , 0 };
	Mat Xc(3, 1, CV_64FC1, unit_x);
	Mat Xw = R_inv*Xc; // world coordinate of camera X axis
	double* xw = (double*)Xw.data;
	double xpan[] = { cos(pan), sin(pan), 0 };

	double roll = acos(xw[0] * xpan[0] + xw[1] * xpan[1] + xw[2] * xpan[2]); // inner product
	if (xw[2] < 0) roll = -roll;

	cout << "pan tilt roll" << endl;
	cout << pan << endl;
	cout << tilt << endl;
	cout << roll << endl;



	// 3D data to 2D image plane Okay.
	std::vector<cv::Point2f> projectedPoints;
	cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

	for (unsigned int i = 0; i < projectedPoints.size(); ++i)
	{
		// 실제 image point 와 3D로 부터 project 한 2D 이미지 좌표를 비교할 수 있습니다. 그리고 오차 구하기 가능합니다. 
	   std::cout << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << std::endl;
	}

	return 0;
}
//

std::vector<cv::Point2f> Generate2DPoints()
{
	std::vector<cv::Point2f> points;

	float x, y;

	x = 308; y = 139;
	points.push_back(cv::Point2f(x, y));
	x = 290; y = 414;
	points.push_back(cv::Point2f(x, y));
	x = 450; y = 123;
	points.push_back(cv::Point2f(x, y));
	x = 455; y = 410;
	points.push_back(cv::Point2f(x, y));
	x = 664; y = 91;
	points.push_back(cv::Point2f(x, y));
	x = 668; y = 408;
	points.push_back(cv::Point2f(x, y));





	for (unsigned int i = 0; i < points.size(); ++i)
	{
		std::cout << points[i] << std::endl;
	}

	return points;
}

// meter to centi
//3D : -0.614177, 2.31638, 0
//- 0.614000, 2.1214, -0.60591
//0.264234, 2.07149, -0.57615
//0.26835, 0.26835, 2.2556

std::vector<cv::Point3f> Generate3DPoints()
{
	std::vector<cv::Point3f> points;


	float x, y, z;

	// cm

	x = 40; y = 31; z = 9;
	points.push_back(cv::Point3f(x, y, z));
	x = 40; y = 31; z = 0.5;
	points.push_back(cv::Point3f(x, y, z));
	x = 63; y = 27; z = 16;
	points.push_back(cv::Point3f(x, y, z));
	x = 63; y = 27; z = 0;
	points.push_back(cv::Point3f(x, y, z));
	x = 58; y = 17.5; z = 16;
	points.push_back(cv::Point3f(x, y, z));
	x = 58; y = 17.5; z = 0;
	points.push_back(cv::Point3f(x, y, z));



	for (unsigned int i = 0; i < points.size(); ++i)
	{
		std::cout << points[i] << std::endl;
	}

	return points;
}