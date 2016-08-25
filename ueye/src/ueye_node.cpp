#include "disparity/ueye_camera.h"
#include "disparity/ueye_exceptions.h"
#include "disparity/exceptions.h"
#include <opencv2/core/core.hpp>
#include "iostream"
#include <string>
#include <sstream>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/photo/photo.hpp"
#include <stdlib.h>

#include <stdio.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

// Camera class from library
CUeye_Camera ueye;
CUeye_Camera ueye_R;

// GET IMAGE
//-----------------------------------------------------------------------------
cv::Mat* get_img(CUeye_Camera& cam)
{
  // Aacquire a single image from the camera
  bool image_ok = false;
  try
  {
    image_ok = cam.get_image();
  }
  catch (CUeyeCameraException & e)
  {
    cout << e.what () << endl;
    return false;
  }
  catch (CUeyeFeatureException & e)
  {
    cout << e.what () << endl;
  }

  int type;
  if(cam.params_.img_bpp ==8) 
	type=CV_8UC1;
  else if(cam.params_.img_bpp ==24 || cam.params_.img_bpp==32)
	type=CV_8UC3;

  cv::Mat* image = NULL;

  if(image_ok)
  {
    image = new cv::Mat(cam.params_.img_height, cam.params_.img_width, type);

    for (int jj = 0; jj < cam.img_data_size_; ++jj)
      image->at<unsigned char>(jj) = (unsigned char)cam.image_data_.at(jj);
  }

  return image;
}

// INITIALIZE CAMERA
// -----------------------------------------------------------------------------
bool init_camera()
{
  // Initialize camera
  cout << "[Camera test]: Trying to open camera connection..." << endl;
  try
  {
    ueye.init_camera();
  }
  catch (CUeyeCameraException & e)
  {
    cout << e.what () << endl;
    return false;
  }
  catch (CUeyeFeatureException & e)
  {
    cout << e.what () << endl;
  }

  cout << "[Camera test]: Trying to open camera connection..." << endl;
  try
  {
    ueye_R.init_camera();
  }
  catch (CUeyeCameraException & e)
  {
    cout << e.what () << endl;
    return false;
  }
  catch (CUeyeFeatureException & e)
  {
    cout << e.what () << endl;
  }

  return true;
}

// LIST AVAILABLE CAMERAS
// -----------------------------------------------------------------------------
bool list_cameras()
{
  // List cameras
  try
  {
    ueye.list_cameras();
  }
  catch (CUeyeCameraException & e)
  {
    cout << e.what () << endl;
    return false;
  }
  catch (CUeyeFeatureException & e)
  {
    cout << e.what () << endl;
  }
  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ueye_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub1 = it.advertise("camera/image_left", 1);
  image_transport::Publisher pub2 = it.advertise("camera/image_right",1);
  image_transport::Publisher pub3 = it.advertise("camera/image_combine",1);

  sensor_msgs::ImagePtr msg1;
  sensor_msgs::ImagePtr msg2;
  sensor_msgs::ImagePtr msg3;

  // Define Initial parameters
  ueye.params_.cameraid     		=1;
  ueye.params_.img_width 		=752;
  ueye.params_.img_height    		=480;
  ueye.params_.img_left      		=-1;
  ueye.params_.img_top       		=-1;
  ueye.params_.fps           		=20;
  ueye.params_.param_mode      		=0;
  ueye.params_.file_str       		="";
  ueye.params_.pixel_clock   		=20;
  ueye.params_.exposure     		=0.2;
  ueye.params_.mirror_updown  		=false;
  ueye.params_.mirror_leftright 	=false;

  ueye_R.params_.cameraid    		=2;
  ueye_R.params_.img_width    		=752;
  ueye_R.params_.img_height  		=480;
  ueye_R.params_.img_left   		=-1;
  ueye_R.params_.img_top       		=-1;
  ueye_R.params_.fps          		=20;
  ueye_R.params_.param_mode  		=1;
  ueye_R.params_.file_str     		="";
  ueye_R.params_.pixel_clock    	=20;
  ueye_R.params_.exposure  		=0.2;
  ueye_R.params_.mirror_updown 		=false;
  ueye_R.params_.mirror_leftright 	=false;

  
  // Initialize camera
  if(!init_camera())
    return false;

  //List cameras to choose which one is going to be tested
  if(!list_cameras())
    return false;

  cv::Mat* frame_L;
  cv::Mat* frame_R;
  Mat img1;
  Mat img2;
  Mat img3;//combined image

  ros::Rate loop_rate(20);
  int64 t = getTickCount();
  ueye.Enable_Event();
  ueye_R.Enable_Event();

  while (nh.ok()) {

    ueye.getExposure();
    ueye_R.getExposure();

/*
    ueye.getExposure();
    ueye_R.set_exposure(ueye.input_exposure);
    ueye_R.getExposure();
    std::cout<<"ueye.input_exposure"<<ueye.input_exposure<<std::endl;
    std::cout<<"ueye_R.input_exposure"<<ueye_R.input_exposure<<std::endl;
*/

    //std::cout<<"exposure"<<ueye.input_exposure<<" "<<ueye_R.input_exposure<<std::endl;    
    //std::cout<<"gain"<<ueye.get_hardware_gain()<<" "<<ueye_R.get_hardware_gain()<<std::endl;

 
    ueye_R.get_hardware_gain();
    int gain = ueye.get_hardware_gain();
    ueye_R.set_hardware_gain(gain);

    int flag1 = ueye.Wait_next_image();
    int flag2 = ueye_R.Wait_next_image();

   // std::cout<<flag1<<flag2<<std::endl;

  //  if(flag1&&flag2)
  //  {
    frame_L                              = get_img(ueye);
    frame_R                              = get_img(ueye_R);
    if(frame_L!=NULL && frame_R!=NULL &&  flag1 && flag2)
    {

	img1 = *frame_L;
	cvtColor(img1,img1,CV_RGB2GRAY);
	img2 = *frame_R;
	cvtColor(img2,img2,CV_RGB2GRAY);
	hconcat(img1, img2, img3);
  	imshow("img1",img1);
  	imshow("img2",img2);
  	char button=waitKey(1);

  	if (button == 'a'){
		std::cout<<"exposure"<<ueye.input_exposure<<" "<<ueye_R.input_exposure<<std::endl;    
    		std::cout<<"gain"<<ueye.get_hardware_gain()<<" "<<ueye_R.get_hardware_gain()<<std::endl;
	}

	msg1 = cv_bridge::CvImage(std_msgs::Header(), "mono8", img1).toImageMsg();
	msg2 = cv_bridge::CvImage(std_msgs::Header(), "mono8", img2).toImageMsg();
	msg3 = cv_bridge::CvImage(std_msgs::Header(), "mono8", img3).toImageMsg();
	pub1.publish(msg1);
	pub2.publish(msg2);
	pub3.publish(msg3);

	frame_L->release();
	frame_R->release();

        t  = getTickCount() - t;
       // std::cout<<"Stereo Matching frequency: " << getTickFrequency()/t<<std::endl;
    }
    else{
        ROS_INFO_STREAM("LOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOST");
    }

    ros::spinOnce();
    loop_rate.sleep();
 }//end of ROS
}
