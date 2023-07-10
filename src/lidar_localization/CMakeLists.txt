cmake_minimum_required(VERSION 3.0.2)
project(lidar_localization)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package (GeographicLib REQUIRED)
include_directories (${GeographicLib_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_localization
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

# include是指包含本软件包下的头文件, ${catkin_INCLUDE_DIRS}是指ROS下其它包的头文件，include需要写在${catkin_INCLUDE_DIRS}前面。
include_directories(include ${catkin_INCLUDE_DIRS})

