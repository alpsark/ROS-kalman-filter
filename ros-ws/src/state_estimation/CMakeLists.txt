cmake_minimum_required(VERSION 2.8.3)
project(state_estimation)


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf tf_conversions pcl_ros pcl_conversions sensor_msgs cv_bridge image_transport vrep_common)
find_package(Boost REQUIRED COMPONENTS system)
#find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vrepscan_to_laserscan
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_executable(state_estimation src/state_estimation.cpp)
target_link_libraries(state_estimation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
