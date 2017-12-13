cmake_minimum_required(VERSION 2.8.3)
project(drogo)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base
  tf
  roscpp
  rospy
  std_msgs
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS visualization_msgs
#  INCLUDE_DIRS include
#  LIBRARIES hypha_racecar
#  CATKIN_DEPENDS geometry_msgs move_base roscp rospy std_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/hypha_racecar.cpp
# )

###-------------------###
###---pp_controller---###
###-------------------###
add_executable(pp_controller src/pp_controller/pp_controller.cpp 
							 src/pp_controller/pp_controller.h)
target_link_libraries(pp_controller ${catkin_LIBRARIES})
#add_dependencies(pp_controller drogo_onBoard_generate_messages_cpp)

###----------###
###---GY85---###
###----------###
add_executable(imu_auto_lucky src/gy85/gy85_pub.cpp src/gy85/gy-85.cpp src/gy85/gy-85.h)
target_link_libraries(imu_auto_lucky ${catkin_LIBRARIES})

###----------------###
###---Controller---###
###----------------###
add_executable(pca9685_lucky_kb src/controller/kb_controller.cpp 
								src/controller/JHPWMPCA9685.cpp 
								src/controller/JHPWMPCA9685.h)
target_link_libraries(pca9685_lucky_kb ${catkin_LIBRARIES})

add_executable(pca9685_lucky_stick src/controller/stick_controller.cpp 
								   src/controller/JHPWMPCA9685.cpp 
								   src/controller/JHPWMPCA9685.h)
target_link_libraries(pca9685_lucky_stick ${catkin_LIBRARIES})

add_executable(pca9685_lucky_nav src/controller/nav_controller.cpp 
								 src/controller/JHPWMPCA9685.cpp 
								 src/controller/JHPWMPCA9685.h)
target_link_libraries(pca9685_lucky_nav ${catkin_LIBRARIES})

add_executable(fuse_cmd_vel src/controller/fuse_cmd_vel.cpp)
target_link_libraries(fuse_cmd_vel ${catkin_LIBRARIES})

###-----------###
###---aslam---###
###-----------###
add_executable(send_goal src/simple_navigation_goals.cpp)
target_link_libraries(send_goal ${catkin_LIBRARIES})

add_executable(aslam_vector_follower src/aslam/aslam_vector_follower.cpp 
					 				 src/aslam/aslam_vector_follower.h)
target_link_libraries(aslam_vector_follower ${catkin_LIBRARIES})








