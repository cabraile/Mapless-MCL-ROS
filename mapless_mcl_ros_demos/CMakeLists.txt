cmake_minimum_required(VERSION 3.1)
project(mapless_mcl_ros_demos)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

# Dependencies
# ------------------

find_package(catkin REQUIRED COMPONENTS
  tf2
  tf2_ros
  rospy
)

# Include
# -----------------

catkin_package(
  INCLUDE_DIRS
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

link_directories()