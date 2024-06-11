cmake_minimum_required(VERSION 3.0.2)
project(hrpsys_ext_rtc)

# Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild2
  roscpp)

# find_package(choreonoid)を2回呼ぶとadd_library cannot create imported target "Choreonoid::Cnoid*" because another target with the same name already exists.エラーになるので、親のCMakeLists.txtで1回だけ呼ぶ.
find_package(choreonoid REQUIRED)

# initialize rtmbuild2 (generate_messages is called)
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
