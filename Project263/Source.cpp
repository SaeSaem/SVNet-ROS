#include <opencv2/highgui.hpp>  
#include <opencv2/imgproc.hpp>  
#include <iostream>  

using namespace cv;
using namespace std;



void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		cout << "���� ���콺 ��ư Ŭ��.. ��ǥ = (" << x << ", " << y << ")" << endl;
	}
}


int main(int argc, char** argv)
{

	Mat img_original, img_gray;

	//�̹��������� �ε��Ͽ� image�� ����  
	img_original = imread("6test.jpg", IMREAD_COLOR);
	if (img_original.empty())
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}


	//�׷��̽����� �̹����� ��ȯ  
	cvtColor(img_original, img_gray, COLOR_BGR2GRAY);

	//������ ����  
	namedWindow("original image", WINDOW_AUTOSIZE);
	namedWindow("gray image", WINDOW_AUTOSIZE);


	//�����쿡 ���  
	imshow("original image", img_original);
	imshow("gray image", img_gray);

	//�����쿡 �ݹ��Լ��� ���
	setMouseCallback("gray image", CallBackFunc, NULL);


	//Ű���� �Է��� �ɶ����� ���  
	waitKey(0);

	return 0;
}
