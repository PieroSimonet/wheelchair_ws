#include "rosneuro_msgs/NeuroFrame.h"
#include "rosneuro_msgs/NeuroOutput.h"
#include "rosneuro_data/NeuroDataTools.hpp"

#include <wtkprocessing/RingBuffer.hpp>

#define SUBTOPIC	"/neurodata"
#define PUBTOPIC	"/neuroprediction"
#define NSAMPLES	32
#define NCHANNELS	16
#define BUFFER_SIZE 512
#define SELECTED_CHANNEL 1

// Global variables

unsigned int n_samples   = NSAMPLES;
unsigned int n_channels  = NCHANNELS;
unsigned int buffer_size = BUFFER_SIZE;
bool newframe = false;

rosneuro_msgs::NeuroOutput neuromsg;
std::vector<float> neurodata;


void on_received_data(const rosneuro_msgs::NeuroFrame::ConstPtr& msg) {
	if(msg->eeg.info.nsamples == n_samples && msg->eeg.info.nchannels == n_channels) {
		newframe = true;
		neurodata = msg->eeg.data;
		//neuromsg.softpredict.info = neuromsg.hardpredict.info = msg->eeg.info;
	}

}

int main(int argc, char** argv) {
	
	// ros initialization
	ros::init(argc, argv, "fakeprocessing");
	
	ros::NodeHandle nh;
	ros::Subscriber sub;
	ros::Publisher  pub;
	ros::Rate r(256);

	std::string stopic		= SUBTOPIC;
	std::string ptopic		= PUBTOPIC;

	Eigen::MatrixXf dmap;
	Eigen::MatrixXd dbuf;
	Eigen::VectorXd dvec;

	wtk::proc::RingBuffer* 	buffer;
	buffer = new wtk::proc::RingBuffer();

	// Subscriber/Publisher initialization
	sub = nh.subscribe(stopic, 1, on_received_data);
	pub = nh.advertise<rosneuro_msgs::NeuroOutput>(ptopic, 1);

	// Buffer and matrix initialization
	buffer->Setup(buffer_size, n_channels);
	dmap = Eigen::MatrixXf::Zero(n_channels, n_samples);
	dbuf = Eigen::MatrixXd::Zero(buffer_size, n_channels);
	dvec = Eigen::VectorXd::Zero(buffer_size);

	double maxval;
	float output;
	while(ros::ok()) {
		ros::spinOnce();
		r.sleep();

		if(newframe == false)
			continue;

		dmap = Eigen::Map<Eigen::MatrixXf>(neurodata.data(), n_channels, n_samples);
		dmap.transposeInPlace();

		buffer->Add(dmap.cast<double>());
	
		if(buffer->IsFull() == false)
			continue;

		buffer->Get(dbuf);

		dvec = dbuf.col(SELECTED_CHANNEL).cwiseAbs();

		maxval = dvec.maxCoeff();
		output = (float)(dvec / maxval).mean();
       

		neuromsg.header.stamp = ros::Time::now();
		neuromsg.softpredict.data = {output, 1-output};
		pub.publish(neuromsg);
		newframe = false;

	}	

	ros::shutdown();
	return 0;
}
