cmake_minimum_required(VERSION 3.0.2)
project(lidar_localization)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  tf
)

find_package (GeographicLib REQUIRED)
include_directories (${GeographicLib_INCLUDE_DIRS})

find_package (PCL REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_localization
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

# include是指包含本软件包下的头文件, ${catkin_INCLUDE_DIRS}是指ROS下其它包的头文件，include需要写在${catkin_INCLUDE_DIRS}前面。
include_directories(include ${catkin_INCLUDE_DIRS})
include(cmake/global_definition.cmake)

# 依赖库
include_directories("/usr/include/eigen3")
set(ALL_TARGET_LIBRARIES "")
list(APPEND ALL_TARGET_LIBRARIES ${PCL_LIBRARIES})
list(APPEND ALL_TARGET_LIBRARIES ${GeographicLib_LIBRARIES})
# 没有手动安装过glog
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)
list(APPEND ALL_TARGET_LIBRARIES ${glog_LIBRARIES})

# 源文件
file(GLOB_RECURSE ALL_SRCS "*.cpp")

add_executable(test_frame_node src/apps/test_frame_node.cpp ${ALL_SRCS})
target_link_libraries(test_frame_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})