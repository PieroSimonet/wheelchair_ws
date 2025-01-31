cmake_minimum_required(VERSION 2.8.3)
project(rosneuro_processing)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
			 rosneuro_data
			 rosneuro_msgs
			 dynamic_reconfigure)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_search_module(WTKPROCESSING REQUIRED wtkprocessing)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

generate_dynamic_reconfigure_options(
	cfg/EogBci.cfg
)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	std_msgs
	rosneuro_data
	rosneuro_msgs
  DEPENDS
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} 
                    ${EIGEN3_INCLUDE_DIRS}
		    ${WTKPROCESSING_INCLUDE_DIRS}
		    include
		    
)

add_definitions(${EIGEN3_DEFINITIONS})


add_library(${PROJECT_NAME}
	src/SmrBci.cpp
	src/EogBci.cpp
)

target_link_libraries(${PROJECT_NAME} 
		     ${catkin_LIBRARIES} 
		     ${WTKPROCESSING_LIBRARIES}
			 ${Eigen3_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} 
	         ${catkin_EXPORTED_TARGETS}
)


#################
## Executables ##
#################

add_executable(test_smrbci src/test_smrbci.cpp)
target_link_libraries(test_smrbci rosneuro_processing)
add_dependencies(test_smrbci ${PROJECT_NAME}_gencfg)

add_executable(test_eogbci src/test_eogbci.cpp)
target_link_libraries(test_eogbci rosneuro_processing)
add_dependencies(test_eogbci ${PROJECT_NAME}_gencfg)

add_executable(test_processing src/test_processing.cpp)
target_link_libraries(test_processing rosneuro_processing)
add_dependencies(test_processing ${PROJECT_NAME}_gencfg)

add_executable(fakeprocessing src/fakeprocessing.cpp)
target_link_libraries(fakeprocessing rosneuro_processing)
add_dependencies(fakeprocessing ${PROJECT_NAME}_gencfg)


