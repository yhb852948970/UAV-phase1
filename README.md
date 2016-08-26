# UAV-phase1

ROS stack for the NTU-ST UAV autonomous navigation project. Phase 1 deliverable.


In this ROS stack, there are three modules, i.e. ueye, stereo_disparity, depth_to_line_extraction.

1. ueye package is the camera driver.

2. stereo_disparity module is in charge of the stereo matching part.

3. depth_to_line_extraction is used to convert the depth message to a ROS laser_scan message.

4. ROS rviz tool is used for visulization.  



Dependency of this stack:

1. Ueye camera official driver, which is provided by IDS and it can be downloaded from this website.

2. iriutils package. This is the driver for IDS ueye camera in ROS.

3. OpenCV 2.4.x. x can be 8-13. For our testing, OpenCV 2.4.12 and OpenCV 2.4.13 are used.

4. ROS indigo. 

5. These algorithms are tested in Ubuntu Linux 14.04.



Usage of this stack:

1. put the fold "UAV-phase1" into the catkin workspace named "catkin_ws" in the home directory. 
If the catkin workspace are not named like this or in the other directory, please amend the path to rviz file in the launch file accordingly.

2. catkin_make

3. roslaunch /path/to/launch/file/ST_UAV.launch

A demo launch file using the data in a rosbag is provided. The name of that launch file is "ST_UAV_demo.launch". In the demo, rosbag is required, but the stereo cameras are not necessary. 
