cmake_minimum_required(VERSION 3.0)
project(hmm_feedback)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
       pluginlib
			 rosneuro_msgs
			 geometry_msgs
)

# Import the neurodraw library
find_path(NDP neurodraw PATHS ${CMAKE_BINARY_DIR})
message(${NDP})
if(NOT EXISTS ${CMAKE_BINARY_DIR}/neurodraw)

  include(ExternalProject)
  ExternalProject_Add(ext_neurodraw
    GIT_REPOSITORY https://github.com/neurorobotics-iaslab/neurodraw
    GIT_TAG main
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/neurodraw
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/neurodraw
  ) 
  
  add_library(neurodraw SHARED IMPORTED)
  add_library(neurochrono SHARED IMPORTED)
  
  set_property(TARGET neurodraw PROPERTY IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/neurodraw/lib/libneurodraw.so)
  set_property(TARGET neurochrono PROPERTY IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/neurodraw/lib/libneurochrono.so)
  
  add_dependencies(neurodraw ext_neurodraw)
  add_dependencies(neurochrono ext_neurodraw)
else()
  add_library(neurodraw SHARED IMPORTED)
  add_library(neurochrono SHARED IMPORTED)
  
  set_property(TARGET neurodraw PROPERTY IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/neurodraw/lib/libneurodraw.so)
  set_property(TARGET neurochrono PROPERTY IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/neurodraw/lib/libneurochrono.so)
endif()

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
          # ${install_dir}/src/ext_neurodraw/include
          ${CMAKE_BINARY_DIR}/neurodraw/include
          )

add_library(${PROJECT_NAME}	
      src/draw/bars.cpp
      src/engine/engineHmmBars.cpp
			)

target_link_libraries(${PROJECT_NAME} 
					  ${catkin_LIBRARIES}
					  neurodraw
            neurochrono 
					 )

add_dependencies(${PROJECT_NAME} 
  ${catkin_EXPORTED_TARGETS}
)

#################
## Executables ##
#################
add_executable(hmm_bars src/bars.cpp)
target_link_libraries(hmm_bars ${PROJECT_NAME}
								 ${catkin_LIBRARIES}
                 neurodraw
                 neurochrono
                 )
