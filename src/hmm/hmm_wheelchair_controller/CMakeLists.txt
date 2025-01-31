cmake_minimum_required(VERSION 3.0)
project(hmm_wheelchiar_controller)

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
	include
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

include_directories(include 
					${catkin_INCLUDE_DIRS}
          )


add_library(${PROJECT_NAME}	
      src/Controller.cpp
			)

target_link_libraries(${PROJECT_NAME} 
					  ${catkin_LIBRARIES}
					 )

add_dependencies(${PROJECT_NAME} 
  ${catkin_EXPORTED_TARGETS}
)

#################
## Executables ##
#################
add_executable(hmm_wheelchiar_controller_node src/controller.cpp)
target_link_libraries(hmm_wheelchiar_controller_node ${PROJECT_NAME}
								 ${catkin_LIBRARIES}
                 )
