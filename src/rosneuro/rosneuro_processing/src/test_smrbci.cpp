#include <ros/ros.h>
#include "rosneuro_processing/SmrBci.hpp"

int main(int argc, char** argv) {

	
	// ros initialization
	ros::init(argc, argv, "smrbci");

	rosneuro::SmrBci smrbci;
	
	if(smrbci.configure()== false) {
		std::cerr<<"SETUP ERROR"<<std::endl;
		return -1;
	}

	ros::Rate r(256);
	while(ros::ok())
	{

		 if(smrbci.Classify() == true) {
			ROS_INFO_ONCE("Classification started"); 
		 } 
	
         ros::spinOnce();
		 r.sleep();
	}

	ros::shutdown();
	return 0;
}
