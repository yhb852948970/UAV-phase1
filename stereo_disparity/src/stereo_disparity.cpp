// ROS headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

// OpenCV headers
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

// system headers
#include <math.h>
#include <iterator>
#include <algorithm>
#include <stdio.h>

#define width 752
#define height 480

using namespace std;
using namespace cv;
/*
double M_1[9] = { 420.6975, 0, 368.2764, 0, 418.8214, 239.7958, 0, 0, 1  };
double M_2[9] = { 424.2353, 0, 376.1645, 0, 422.5504, 237.5235, 0, 0, 1  };
double D_1[4] = {  -0.2930, 0.0839, -9.8567e-04, 3.0856e-03  };
double D_2[4] = {  -0.2991, 0.0916, -1.6000e-03, 5.4858e-04  };
double r_data[9] = { 0.9998, -0.0051, 0.0199, 0.0051, 1.0000, -9.5538e-05, -0.0199, 1.9647e-04, 0.9998};
double t_data[3] = { -2.995894e+02, -0.7549, 1.9833 };
*/

double r_data[9] = {0.999851516004708, -0.00498457045275044, 0.0164954539340047, 0.00497160703876063, 0.999987299688415, 0.000826792189086883, -0.0164993656405163, -0.000744660508793523, 0.999863598904464};
double t_data[3] = {-299.268590358311,	-0.641667329520781,	1.09821941809761};
double M_1[9] = { 421.199009407044, 0.412760990698931, 366.699973333038, 0, 421.655468122576,  239.600056673828, 0, 0, 1 };
double M_2[9] = {423.526696160263, -0.123652334236988, 375.431005950529, 0, 423.751038507976, 237.088401774212, 0, 0, 1  };
double D_1[4] = {-0.307057933010874,	0.120679939434720, -0.000340441435229576,	-0.000347173827361101};
double D_2[4] = {-0.306923298818246,	0.121786424922333, -0.000810100722834597,	0.000975575619740761};

class stereo_disparity
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber img_combine;
  image_transport::Publisher img_disparity;
  image_transport::Publisher img_depth;

  Size img_size;
  Rect roi_left;
  Rect roi_right;
  Rect roi_half;

  Mat img_left, img_right;
  Mat img_left_rect, img_right_rect;
  Mat img_left_half, img_right_half;

  Mat M1, D1, M2, D2;
  Mat R, T;

  Mat map11, map12, map21, map22;
  Mat disp, disp8, disp_ori, depth_32F;
  Mat bgr[3];

  Rect roi1, roi2;
  Mat Q;
  Mat R1, P1, R2, P2;

  StereoSGBM sgbm;
  int cn;
  float depth;

  public:
  stereo_disparity() : 	it(nh) {
	roi_left = Rect(0, 0, width, height);
	roi_right = Rect(width, 0, width, height);
	roi_half = Rect(0, height/4, width, height/2);
	img_size = Size(width, height);

    	M1= cv::Mat(3, 3, CV_64FC1, &M_1);
    	M2 = cv::Mat(3, 3, CV_64FC1, &M_2);
    	D1 = cv::Mat(1, 4, CV_64FC1, &D_1);
    	D2 = cv::Mat(1, 4, CV_64FC1, &D_2);
    	R = cv::Mat(3, 3, CV_64FC1, &r_data);
    	T = cv::Mat(3, 1, CV_64FC1, &t_data);

	cn = 1;
	sgbm.preFilterCap = 63; //34
	sgbm.SADWindowSize = 5; //1
	sgbm.P1 = 20*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
  	sgbm.disp12MaxDiff = 1; // cannot set it to 0
	sgbm.minDisparity = 3;
	sgbm.numberOfDisparities = 64;
	sgbm.uniquenessRatio = 10;
	sgbm.speckleWindowSize = 125;
	sgbm.speckleRange = 1; // will check this param

	//sgbm.fullDP = alg == STEREO_SGBM;

	depth =0;

	stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0, img_size, &roi1, &roi2 );
	initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
	initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
cout << "Q = " << Q << endl << endl;
cout << "P1 = " << P1 << endl << endl;
cout << "P2 = " << P2 << endl << endl;
  	img_combine = it.subscribe("/camera/image_combine", 1, &stereo_disparity::imageCallback, this);
	img_disparity = it.advertise("/stereo/disparity", 1);
	img_depth = it.advertise("/stereo/depth", 1);
  }

  ~stereo_disparity(){
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg){

	cv_bridge::CvImagePtr cv_ptr;	// opencv Mat pointer;

    	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
    	}

    	catch (cv_bridge::Exception& e) {
      	  	ROS_ERROR("cv_bridge exception: %s", e.what());
          	return;
    	}

	// cv_ptr->image   ----> Mat image in opencv

	img_left = cv_ptr->image(roi_left);
	img_right = cv_ptr->image(roi_right);
	remap(img_left, img_left_rect, map11, map12, INTER_LINEAR);
	remap(img_right, img_right_rect, map21, map22, INTER_LINEAR);
	img_left_half = img_left_rect(roi_half);
	img_right_half = img_right_rect(roi_half);

	int64 t = getTickCount();
	sgbm(img_left_half, img_right_half, disp);
	disp.convertTo(disp8, CV_8U, 255/(sgbm.numberOfDisparities*16.));
	disp_ori = disp / 16;
	reprojectImageTo3D(disp_ori, depth_32F, Q);
	split(depth_32F, bgr);
	bgr[2] = bgr[2] / 1000;
//	depth = disp.at<short>(height/4,width/2);
//	depth = 420*0.3*16/disp.at<short>(disp.rows/2, disp.cols/2);
	depth = bgr[2].at<float>(height/4,width/2);

	ROS_INFO_STREAM("depth of the center point is " << depth << " m." );
	t = getTickCount() - t;
	ROS_INFO_STREAM("Stereo Matching frequency: " << getTickFrequency()/t);

	sensor_msgs::ImagePtr disparity = cv_bridge::CvImage(std_msgs::Header(), "mono8", disp8).toImageMsg();
        img_disparity.publish(disparity);
	sensor_msgs::ImagePtr depth = cv_bridge::CvImage(std_msgs::Header(), "32FC1",bgr[2]).toImageMsg();
        img_depth.publish(depth);

  }	// for imageCallback
};	// for the class

int main(int argc, char** argv){

	ros::init(argc, argv, "stereo_disparity");	// The third argument is the cpp filename
	stereo_disparity stereo;
	ros::spin();
	return 0;
}
