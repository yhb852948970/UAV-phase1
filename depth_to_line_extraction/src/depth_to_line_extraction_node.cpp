// ROS headers
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Quaternion.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>

// OpenCV headers
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <iterator>
#include <math.h>
#include <vector>

#define NUM_LINES_H 11
#define NUM_LINES_V 5
#define NUM_PIXELS 9
#define MAX_DETECT_DISTANCE 30
#define MIN_DETECT_DISTANCE 1.5
#define HOR_ANGLE (88.88 / 180 * 3.14)
#define VER_ANGLE (32. / 180 * 3.14)

// median function
double getMedian(double *arry, int size) {
  std::sort(arry, arry + size);
  if ((arry[size / 2] == arry[size / 2 - 1]) ||
      (arry[size / 2] == arry[size / 2 + 1]))
    return (arry[size / 2]);
  else
    return 0;
}

// function mode
double getMode(double new_array[], int num) {
  double *ipRepetition = new double[num];

  for (int i = 0; i < num; i++) {
    ipRepetition[i] = 0; // initialize each element to 0
    int j = 0;           //
    while ((j < i) && (new_array[i] != new_array[j])) {
      if (new_array[i] != new_array[j]) {
        j++;
      }
    }
    (ipRepetition[j])++;
  }
  int iMaxRepeat = 0;
  for (int i = 1; i < num; i++) {
    if (ipRepetition[i] > ipRepetition[iMaxRepeat]) {
      iMaxRepeat = i;
    }
  }
  // cout<< "The mode is " << new_array[iMaxRepeat] << endl;
  delete[] ipRepetition;
  return new_array[iMaxRepeat];
}

/*
// Global variables
double IMU_x, IMU_y, IMU_z, IMU_w;

void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("Imu Seq: [%d]", msg->header.seq);
  ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]",
msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
  IMU_x = msg->orientation.x;
  IMU_y = msg->orientation.y;
  IMU_z = msg->orientation.z;
  IMU_w = msg->orientation.w;
}
*/

class depth_to_line_extraction {
  ros::NodeHandle nh;

  ros::Publisher Vscan_pub;
  ros::Publisher Hscan_pub;

  //  ros::Subscriber IMU_sub;

  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

  unsigned int Vnum_readings;
  unsigned int Hnum_readings;
  double laser_frequency;
  tf::TransformBroadcaster broadcaster;
  tf::Transform transform;

public:
  depth_to_line_extraction() : it(nh) {

    image_sub = it.subscribe("/stereo/depth", 1,
                             &depth_to_line_extraction::imageCallback, this);
    Vscan_pub = nh.advertise<sensor_msgs::LaserScan>("Vscan", 1);
    Hscan_pub = nh.advertise<sensor_msgs::LaserScan>("Hscan", 1);
    // IMU_sub = nh.subscribe("imu_data", 1, &IMUCallback);

    laser_frequency = 1;
  }

  ~depth_to_line_extraction() {}

  void imageCallback(const sensor_msgs::ImageConstPtr &msg) {

    cv_bridge::CvImagePtr cv_ptr; // opencv Mat pointer;

    try {
      cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }

    catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    Vnum_readings = cv_ptr->image.rows;
    Hnum_readings = cv_ptr->image.cols;
    double Img_col_filter[Hnum_readings][NUM_LINES_H];
    double Img_col_filter_V[Vnum_readings][NUM_LINES_V];
    double Img_row_filter[Hnum_readings - NUM_PIXELS + 1][NUM_PIXELS];

    ros::Time scan_time = ros::Time::now(); // lookup the ros::Time class

    sensor_msgs::LaserScan Hscan;
    Hscan.header.stamp = scan_time;
    Hscan.header.frame_id = "Hlaser_frame";
    Hscan.angle_min = HOR_ANGLE / 2 * (-1);
    Hscan.angle_max = HOR_ANGLE / 2;
    Hscan.angle_increment = HOR_ANGLE / Hnum_readings;
    Hscan.time_increment = 0; //(1 / laser_frequency) / (Hnum_readings);
    Hscan.range_min = MIN_DETECT_DISTANCE;
    Hscan.range_max = MAX_DETECT_DISTANCE;
    Hscan.ranges.resize((int)(Hnum_readings));

    sensor_msgs::LaserScan Vscan;
    Vscan.header.stamp = scan_time;
    Vscan.header.frame_id = "Vlaser_frame";
    Vscan.angle_min = VER_ANGLE / 2 * (-1);
    Vscan.angle_max = VER_ANGLE / 2;
    Vscan.angle_increment = VER_ANGLE / Vnum_readings;
    Vscan.time_increment = 0; //(1 / laser_frequency) / (Vnum_readings);
    Vscan.range_min = MIN_DETECT_DISTANCE;
    Vscan.range_max = MAX_DETECT_DISTANCE;
    Vscan.ranges.resize((int)(Vnum_readings));

    // x-axis for horizontal; y-axis for vertical
    // horizontal
    int y = (int)(Vnum_readings / 2); // get y value from imu need to calibrate
    int temp;

    for (int i = 0; i < NUM_LINES_H; i++)
      for (int x = 0; x < Hnum_readings; x++) {
        // transposed in Img_col_filte
        temp = cv_ptr->image.at<float>(y - (int)(NUM_LINES_H / 2) + i, x);
        Img_col_filter[x][i] = (temp < MAX_DETECT_DISTANCE) ? temp : 0;
      }

    for (int x = 0; x < Hnum_readings; x++) {
      // Hscan.ranges[Hnum_readings-x-1] = getMedian( Img_col_filter[x],
      // NUM_LINES_H);
      Hscan.ranges[Hnum_readings - x - 1] =
          getMode(Img_col_filter[x], NUM_LINES_H);
    }

    // for( int x = 0; x < Hnum_readings - NUM_PIXELS + 1; x++){
    //     for( int j = 0; j < NUM_PIXELS; j++){
    //         Img_row_filter[x][j] = Hscan.ranges[x+j];
    //     }
    //         Hscan.ranges[x] = getMedian( Img_row_filter[x], NUM_PIXELS);
    // }

    for (int x = 0; x < Hnum_readings; x++) {
      Hscan.ranges[x] =
          Hscan.ranges[x] / cos(Hscan.angle_min + x * Hscan.angle_increment);
      // Hscan.ranges[x] = (Hscan.ranges[x] < MAX_DETECT_DISTANCE) ? Hscan.ranges[x] : 0;
      // change to only 1 decimal point;
      // Hscan.ranges[x] = (int)(Hscan.ranges[x]*20)/20.0;
      // std::cout << Hscan.ranges[x] << std::endl;
      if (x < 26 || x > 725) {
        Hscan.ranges[x] = 0;
      }
    }

    // vertical
    int x = (int)(Hnum_readings / 2); // get x value from imu need to calibrate
    // for( int y = 0; y < Vnum_readings; ++y){
    //     Vscan.ranges[y] = cv_ptr->image.at<float>(y, x);
    //     Vscan.ranges[y] =
    //     Vscan.ranges[y]/cos(Vscan.angle_min+y*Vscan.angle_increment);
    // }

    for (int i = 0; i < NUM_LINES_V; i++)
      for (int y = 0; y < Vnum_readings; y++) {
        // transposed in Img_col_filte
        temp = cv_ptr->image.at<float>(y, x - (int)(NUM_LINES_V / 2) + i);
        Img_col_filter_V[y][i] = (temp < MAX_DETECT_DISTANCE) ? temp : 0;
      }

    for (int y = 0; y < Vnum_readings; y++) {
      // Vscan.ranges[y] = getMedian( Img_col_filter[x], NUM_LINES_H);
      Vscan.ranges[y] = getMode(Img_col_filter_V[y], NUM_LINES_V);
      Vscan.ranges[y] =
          Vscan.ranges[y] / cos(Vscan.angle_min + y * Vscan.angle_increment);
    }

    // Add a frame
    // set Hlaser_frame at the origion
    // transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    // transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    // broadcaster.sendTransform(tf::StampedTransform(transform,
    // ros::Time::now(), "world", "Hlaser_frame"));

    // Tf between Hlaser and Vlaser: no translation, 180 deg rotation
    tf::Quaternion qut = tf::createQuaternionFromRPY(-3.14 / 2, 0, 0);
    broadcaster.sendTransform(
        tf::StampedTransform(tf::Transform(qut, tf::Vector3(0.0, 0.0, 0.0)),
                             ros::Time::now(), "Hlaser_frame", "Vlaser_frame"));

    Vscan_pub.publish(Vscan);
    Hscan_pub.publish(Hscan);

  } // for imageCallback
};  // for the class

int main(int argc, char **argv) {
  ros::init(
      argc, argv,
      "depth_to_line_extraction"); // The third argument is the cpp filename
  depth_to_line_extraction dtle;
  ros::spin();
  return 0;
}
