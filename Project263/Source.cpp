#include <opencv2/highgui.hpp>  
#include <opencv2/imgproc.hpp>  
#include <iostream>  

using namespace cv;
using namespace std;



void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		cout << "왼쪽 마우스 버튼 클릭.. 좌표 = (" << x << ", " << y << ")" << endl;
	}
}


int main(int argc, char** argv)
{

	Mat img_original, img_gray;

	//이미지파일을 로드하여 image에 저장  
	img_original = imread("6test.jpg", IMREAD_COLOR);
	if (img_original.empty())
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}


	//그레이스케일 이미지로 변환  
	cvtColor(img_original, img_gray, COLOR_BGR2GRAY);

	//윈도우 생성  
	namedWindow("original image", WINDOW_AUTOSIZE);
	namedWindow("gray image", WINDOW_AUTOSIZE);


	//윈도우에 출력  
	imshow("original image", img_original);
	imshow("gray image", img_gray);

	//윈도우에 콜백함수를 등록
	setMouseCallback("gray image", CallBackFunc, NULL);


	//키보드 입력이 될때까지 대기  
	waitKey(0);

	return 0;
}
