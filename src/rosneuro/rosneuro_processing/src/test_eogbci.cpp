#include <ros/ros.h>
#include "rosneuro_processing/EogBci.hpp"

int main(int argc, char** argv) {

	
	// ros initialization
	ros::init(argc, argv, "eogbci");

	rosneuro::EogBci eogbci;
	
	if(eogbci.configure()== false) {
		std::cerr<<"SETUP ERROR"<<std::endl;
		return -1;
	}

	ros::Rate r(256);
	while(ros::ok())
	{
		 if(eogbci.Apply() == true) {
			ROS_INFO_ONCE("Eog detection started"); 
		 }

		if(eogbci.HasArtifacts(1) == true) {
		       ROS_INFO_ONCE("Eog detected"); 
	
		}

	

         ros::spinOnce();
		 r.sleep();
	}

	ros::shutdown();
	return 0;
}
