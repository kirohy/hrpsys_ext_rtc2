cmake_minimum_required(VERSION 3.0.2)
project(hrpsys_ext_rtc)

# Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

 # rtmbuild has non exist dependency
# (ROSBUILD_genmsg_cpp) and (RTMBUILD_${PROJECT_NAME}_genidl)
cmake_policy(SET CMP0046 OLD)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild2
  roscpp)

# initialize rtmbuild (generate_messages is called)
rtmbuild2_init()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES whole_body_master_slave_choreonoid
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# generate idl
rtmbuild2_genidl()

# generate bridge
rtmbuild2_genbridge()

###########
## Build ##
###########

add_subdirectory(rtc)
