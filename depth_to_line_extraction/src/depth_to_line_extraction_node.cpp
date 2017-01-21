// TO DO
// 1. Cannot only extract one line, not robust
// 2. How to use the IMU reading to find the correct line.

// ROS headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

// OpenCV headers
#include <opencv2/imgproc/imgproc.hpp>

#include <math.h>
#include <iterator>
#include <algorithm>

// Global variables
double IMU_x, IMU_y, IMU_z, IMU_w;

void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("Imu Seq: [%d]", msg->header.seq);
  ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
  IMU_x = msg->orientation.x;
  IMU_y = msg->orientation.y;
  IMU_z = msg->orientation.z;
  IMU_w = msg->orientation.w;
}


class depth_to_line_extraction
{
  ros::NodeHandle nh;

  ros::Publisher Vscan_pub;
  ros::Publisher Hscan_pub;
  ros::Subscriber IMU_sub;
// The next two lines are ZED subscriber
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

  unsigned int Vnum_readings;
  unsigned int Hnum_readings;
  double laser_frequency;	// Based on the fps of depth image from ZED
  tf::TransformBroadcaster broadcaster;
  tf::Transform transform;

  public:
  depth_to_line_extraction() : it(nh) {

  	image_sub = it.subscribe("/camera/depth/image_rect_color", 1, &depth_to_line_extraction::imageCallback, this);
  	Vscan_pub = nh.advertise<sensor_msgs::LaserScan>("Vscan", 1);
  	Hscan_pub = nh.advertise<sensor_msgs::LaserScan>("Hscan", 1);
	//IMU_sub = nh.subscribe("imu_data", 1, &IMUCallback);
   
  	laser_frequency = 40;
  }

  ~depth_to_line_extraction(){
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    	
	cv_bridge::CvImagePtr cv_ptr;	// opencv Mat pointer;

    	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    	}

    	catch (cv_bridge::Exception& e) {
      	  	ROS_ERROR("cv_bridge exception: %s", e.what());
          	return;
    	}

	Vnum_readings = cv_ptr->image.rows;
	Hnum_readings = cv_ptr->image.cols;
 
   	ros::Time scan_time = ros::Time::now();	// lookup the ros::Time class
   	
   	sensor_msgs::LaserScan Hscan;
   	Hscan.header.stamp = scan_time;
   	Hscan.header.frame_id = "Hlaser_frame";
   	Hscan.angle_min = -0.43;
   	Hscan.angle_max = 0.43;
   	Hscan.angle_increment = 0.86 / Hnum_readings;
   	Hscan.time_increment = 0;//(1 / laser_frequency) / (Hnum_readings);
   	Hscan.range_min = 0;
   	Hscan.range_max = 20.0;
   	Hscan.ranges.resize((int)(Hnum_readings));

	sensor_msgs::LaserScan Vscan;
   	Vscan.header.stamp = scan_time;
   	Vscan.header.frame_id = "Vlaser_frame";
   	Vscan.angle_min = -0.33;	// -45 deg to 45 deg
   	Vscan.angle_max = 0.33;
   	Vscan.angle_increment = 0.66 / Vnum_readings;
   	Vscan.time_increment = 0;//(1 / laser_frequency) / (Vnum_readings);
   	Vscan.range_min = 0;	
   	Vscan.range_max = 20.0;
   	Vscan.ranges.resize((int)(Vnum_readings));
 
  
// x-axis for horizontal; y-axis for vertical
	float temp;
        //horizontal   	
        int y=(int)(Vnum_readings/2); //get y value from imu need to calibrate
   	for( int x = 0; x <Hnum_readings; x++){
	    //temp = cv_ptr->image.at<float>(y, x);
      	    Hscan.ranges[Hnum_readings-1-x] = cv_ptr->image.at<float>(y, x)>0 ? cv_ptr->image.at<float>(y, x) : 0;
	    //Hscan.ranges[x] = 5;
	    Hscan.ranges[x] = Hscan.ranges[x]/cos(Hscan.angle_min+x*Hscan.angle_increment);
	    // If there are obstacles in the range [0-10], output is 1; else 0;
	    //Hscan.ranges[x] = (temp > 0 && temp < 10) ? 1 : 0;
            //input_img.at<uchar>(y, x)=255;
            //Hscan.intensities[x] = Hscan.ranges[x];
        }
     
        // vertical
        int x=(int)(Hnum_readings/2); //get x value from imu need to calibrate
        for( int y = 0; y < Vnum_readings; ++y){
	    //temp = cv_ptr->image.at<float>(y, x);
            Vscan.ranges[y] = cv_ptr->image.at<float>(y, x)>0 ? cv_ptr->image.at<float>(y, x) : 0;
	    Vscan.ranges[y] = Vscan.ranges[y]/cos(Vscan.angle_min+y*Vscan.angle_increment);
        }
	
	// Add a frame

	// set Hlaser_frame at the origion
	//transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	//transform.setRotation(tf::Quaternion(0, 0, 0, 1));
	//broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Hlaser_frame")); 

	// Tf between Hlaser and Vlaser: no translation, 180 deg rotation
	tf::Quaternion qut=tf::createQuaternionFromRPY(-3.14/2, 0, 0);
        broadcaster.sendTransform(
        tf::StampedTransform(
        tf::Transform(qut, tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"Hlaser_frame", "Vlaser_frame"));

        Vscan_pub.publish(Vscan);
        Hscan_pub.publish(Hscan);

  }	// for imageCallback
};	// for the class

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_to_line_extraction");	// The third argument is the cpp filename
  depth_to_line_extraction dtle;
  ros::spin();
  return 0;
}
