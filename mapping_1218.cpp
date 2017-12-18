﻿#pragma warning(disable:4996)

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

std::vector<cv::Point3f> read_pcd();
void display_image();
std::vector<cv::Point2f> getRT();
void pcl_to_opencv();


std::vector<cv::Point2f> Generate2DPoints();
std::vector<cv::Point3f> Generate3DPoints();

std::vector<cv::Point2f> twoDpoints;
Mat image;

int main() {
	//read_pcd();
	
	twoDpoints = getRT();
	display_image();

	return 0;
}

void pcl_to_opencv() {

	return;

}

std::vector<cv::Point2f> getRT() {
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


	//Mat src, dst;
	//undistort(src, dst, cameraMatrix, distCoeffs);

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

	int Cx = cameraMatrix.at<double>(0, 0)*X / Z + cameraMatrix.at<double>(0, 2);
	int Cy = cameraMatrix.at<double>(1, 1)*Y / Z + cameraMatrix.at<double>(1, 2);

	cout << "Cx:" << Cx << " Cy:" << Cy << endl;

	cout << endl;
	std::vector<cv::Point2f> projectedPoints;
	cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

	for (unsigned int i = 0; i < projectedPoints.size(); ++i)
	{
		// 실제 image point 와 3D로 부터 project 한 2D 이미지 좌표를 비교할 수 있습니다. 그리고 오차 구하기 가능합니다. 
		std::cout << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << std::endl;
	}

	// 얻음점에서 display 하도록 현재 projectedPoints[i] 가 6개 점이 들어 있습니다. 이에 대해서 수정하도록 합니다. 빨간색 으로 표시, 데이터 접근
	return projectedPoints;
}

void display_image() {
	
	// LOAD image
	image = imread("bandicam 2017-12-12 15-46-32-142.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file "image.jpg".
																				   //This file "image.jpg" should be in the project folder.
																	   //Else provide full address : "D:/images/image.jpg"
	if (!image.data)  // Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
		return ;
	}
	
	int x, y;
	for (int i = 0; i < twoDpoints.size(); i++) {
		x = twoDpoints[i].x;
		y = twoDpoints[i].y;

		circle(image, Point(x, y), 1, Scalar(0, 0, 255), 2);
	//	image.at<Vec3b>(x, y)[2]= 255;
	}

	vector<Point2f> original = Generate2DPoints();
	for (int i = 0; i < original.size(); i++) {
		x = original[i].x;
		y = original[i].y;

		circle(image, Point(x, y), 1, Scalar(255, 0, 0), 2);
		//	image.at<Vec3b>(x, y)[2]= 255;
	}

	
	//DISPLAY image
	namedWindow("window", CV_WINDOW_AUTOSIZE); // Create a window for display.
	imshow("window", image); // Show our image inside it.

	cout << "image size: " << image.rows << " x " << image.cols << endl;

	waitKey(0);
	return;
}

std::vector<cv::Point3f> read_pcd()
{
	std::vector<cv::Point3f> points2;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("xzFiltered_fixed.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return points2;
	}
	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from test_pcd.pcd with the following fields: "
		<< std::endl;
	
//	std::vector<cv::Point3f> points2;
	int x2, y2, z2;

	for (size_t i = 0; i < cloud->points.size(); ++i) {
//		std::cout << "    " << cloud->points[i].x
//			<< " " << cloud->points[i].y
//			<< " " << cloud->points[i].z << std::endl;
		
		//새로운 point2 만들어서 대입하기 pcl data type to opencv data type
		x2 = cloud->points[i].x;
		y2 = cloud->points[i].y;
		z2 = cloud->points[i].z;
		points2.push_back(cv::Point3f(x2, y2, z2));
	}

	cout << points2.size() << endl;
	return points2;
}

// 5496 데이터에 대해서 opencv 함수를 사용해서 맵핑하도록 하겠습니다.
// 1. opecv include 하기.
// 2. read 이것을 함수화로 만들어보도록 하겠습니다.
// 3. 함수화로 모두 만들어서 하나의 main 문으로 사용되도록 하겠습니다.
// 4. 지역, 글로벌 변수 선언 주의할 것.

std::vector<cv::Point2f> Generate2DPoints()
{
	std::vector<cv::Point2f> points;

	float x, y;

	x = 419; y = 104;
	points.push_back(cv::Point2f(x, y));
	x = 429; y = 661;
	points.push_back(cv::Point2f(x, y));
	x = 818; y = 105;
	points.push_back(cv::Point2f(x, y));
	x = 799; y = 676;
	points.push_back(cv::Point2f(x, y));
	x = 965; y = 116;
	points.push_back(cv::Point2f(x, y));
	x = 971; y = 463;
	points.push_back(cv::Point2f(x, y));
	x = 1175; y = 116;
	points.push_back(cv::Point2f(x, y));
	x = 1185; y = 457;
	points.push_back(cv::Point2f(x, y));


	for (unsigned int i = 0; i < points.size(); ++i)
	{
		std::cout << points[i] << std::endl;
	}
	return points;
}

// meter to centi

std::vector<cv::Point3f> Generate3DPoints()
{
	std::vector<cv::Point3f> points;


	float x, y, z;

	// cm

	x = 172; y = 44; z = 20;
	points.push_back(cv::Point3f(x, y, z));
	x = 171; y = 43; z = -58;
	points.push_back(cv::Point3f(x, y, z));
	x = 175; y = -18; z = 20;
	points.push_back(cv::Point3f(x, y, z));
	x = 174; y = -17; z = -58;
	points.push_back(cv::Point3f(x, y, z));

	x = 289; y = -71; z = 17;
	points.push_back(cv::Point3f(x, y, z));
	x = 275; y = -69; z = -63;
	points.push_back(cv::Point3f(x, y, z));
	x = 281; y = -129; z = 17;
	points.push_back(cv::Point3f(x, y, z));
	x = 268; y = -128; z = -66;
	points.push_back(cv::Point3f(x, y, z));



	for (unsigned int i = 0; i < points.size(); ++i)
	{
		std::cout << points[i] << std::endl;
	}

	return points;
}