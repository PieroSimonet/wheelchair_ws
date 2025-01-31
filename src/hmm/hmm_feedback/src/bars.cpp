#include <ros/ros.h>
#include "hmm_feedback/bars.h"
//#include "hmm_feedback/EngineHmmbars.h"


int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "smr_bars");

	rosneuro::bars bars;

	if(bars.configure() == false) {
		ROS_ERROR("Wheel configuration failed");
		ros::shutdown();
		return 1;
	}

	bars.run();

	ros::shutdown();
	

	return 0;

}
