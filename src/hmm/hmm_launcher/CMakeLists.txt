cmake_minimum_required(VERSION 3.0)
project(hmm_launcher)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
       pluginlib
			 rosneuro_msgs
			 geometry_msgs
)

#generate_dynamic_reconfigure_options(
#  cfg/SmrWeelBars.cfg
#)     
 
catkin_package(
  INCLUDE_DIRS 
#	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	std_msgs
	rosneuro_msgs
	geometry_msgs
  DEPENDS
) 
 
 
###########
## Build ##
###########

#################
## Executables ##
#################
