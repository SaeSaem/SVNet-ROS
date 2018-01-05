#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

static const string OPENCV_WINDOW = "Raw Image window";
static const string OPENCV_WINDOW_1 = "Edge Detection";

Mat cameraMatrix(3, 3, DataType<double>::type);
Mat distCoeffs(4, 1, DataType<double>::type);
Mat rvec(3, 1, DataType<double>::type);
Mat tvec(3, 1, DataType<double>::type);
Mat image, drawImg, undistort_img;
vector<Point2f> save2Dpoint;
double x, y;


vector<Point2f> get2Dpoint();
Mat display_image(Mat image);
void param_init();

class cloud
{
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Publisher pcl_pub;
  int cloud_count = 0;

public:
  cloud()
 {
    // Subscribe to input video feed and publish output video feed
    pcl_sub = nh.subscribe("Sensor/points", 10, &cloud::cloudCB, this);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered", 1);
 }

  ~cloud()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  
  void cloudCB(const sensor_msgs::PointCloud2& input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
        sensor_msgs::PointCloud2 output;

        pcl::fromROSMsg(input, cloud);
	
	std::vector<cv::Point3f> points2;
	double x, y, z;
	for (size_t i = 0; i < cloud.points.size(); ++i) {

		x = cloud.points[i].x;
		y = cloud.points[i].y;
		z = cloud.points[i].z;
		points2.push_back(cv::Point3f(x, y, z));
	}

	vector<Point2f> projectedPoints2;
	projectPoints(points2, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints2);
	save2Dpoint = projectedPoints2;
	
        cout << "cloud num: "<< cloud_count << endl;
	cloud_count++;	

        pcl::toROSMsg(cloud_filtered, output);
    	output.header.frame_id = "point_cloud";

        pcl_pub.publish(output);
    }	
};


class fusion
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  int img_count = 0;
  
  int loop_count = 0;
  int cloud_count = 0;
  //ros::NodeHandle nh;
  //ros::Subscriber pcl_sub;
  //ros::Publisher pcl_pub;

public:
  fusion()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
   // pcl_sub = nh.subscribe("Sensor/points", 10, &fusion::cloudCB, this);
   // pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered", 1);    

    image_sub_ = it_.subscribe("/undistort_image", 1, &fusion::imageCb, this);
    image_pub_ = it_.advertise("/edge_detector/raw_image", 1);
    cv::namedWindow(OPENCV_WINDOW);
 //   cout << "loop_count: " << loop_count << endl;
 //   loop_count++;
  }

  ~fusion()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    namespace enc = sensor_msgs::image_encodings;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 400 && cv_ptr->image.cols > 600){


        if(save2Dpoint.size()>1){
	for (int i = 0; i < save2Dpoint.size(); i++) {
		x = save2Dpoint[i].x;
		y = save2Dpoint[i].y;
		circle(cv_ptr->image, Point(x, y), 1, Scalar(0, 0, 255), -1);
		//cout << "x: " << x << endl;
		//cout << "y: " << y << endl;
	}
	imshow(OPENCV_WINDOW, cv_ptr->image);	
	waitKey(1);

	cout << "image num: " << img_count << endl;
	img_count++;

 	image_pub_.publish(cv_ptr->toImageMsg());
	}
     }
  }
};



main(int argc, char** argv)
{
    param_init();
    ros::init(argc, argv, "pcl_and_opencv");
    ROS_INFO("Started two Node");
    fusion ic;
    cloud jc;

    ros::spin();

    return 0;
}

void param_init() {
	cameraMatrix.at<double>(0, 0) = 1173.861610;   // fx
	cameraMatrix.at<double>(0, 1) = 0;
	cameraMatrix.at<double>(0, 2) = 665.778565;    // cx

	cameraMatrix.at<double>(1, 0) = 0;
	cameraMatrix.at<double>(1, 1) = 1171.727336;    // fy
	cameraMatrix.at<double>(1, 2) = 352.757616;     // cy

	cameraMatrix.at<double>(2, 0) = 0;
	cameraMatrix.at<double>(2, 1) = 0;
	cameraMatrix.at<double>(2, 2) = 1;

	distCoeffs.at<double>(0) = -0.281792;
	distCoeffs.at<double>(1) = 0.115447;
	distCoeffs.at<double>(2) = 0.000875;
	distCoeffs.at<double>(3) = -0.001135;

	rvec.at<double>(0) = 1.285994831387268;
	rvec.at<double>(1) = -1.202461389136498;
	rvec.at<double>(2) = 1.180007709399751;

	tvec.at<double>(0) = -0.0003491853220207097;
	tvec.at<double>(1) = 0.2041162796497789;
	tvec.at<double>(2) = -0.03691034276389026;
}
