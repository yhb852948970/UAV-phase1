cmake_minimum_required(VERSION 2.8.3)
project(depth_to_line_extraction)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  cv_bridge
  image_transport
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
)
find_package(OpenCV REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
   #INCLUDE_DIRS include
   #LIBRARIES depth_to_line_extraction
   CATKIN_DEPENDS common_msgs cv_bridge geometry image_transport pcl_ros roscpp sensor_msgs std_msgs tf
   DEPENDS system_lib
)

include_directories( ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )
add_executable(depth_to_line_extraction_node src/depth_to_line_extraction_node.cpp)
target_link_libraries(depth_to_line_extraction_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
