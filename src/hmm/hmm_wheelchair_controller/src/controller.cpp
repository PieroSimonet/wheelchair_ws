#include <ros/ros.h>
#include "hmm_wheelchair_controller/controller.h"


int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "controller");

	rosneuro::controller controller;

	if(controller.configure() == false) {
		ROS_ERROR("Wheel configuration failed");
		ros::shutdown();
		return 1;
	}

	controller.run();

	ros::shutdown();
	

	return 0;

}
