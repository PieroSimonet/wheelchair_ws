#ifndef SMR_BCI_CPP
#define SMR_BCI_CPP

#include "rosneuro_processing/SmrBci.hpp"


namespace rosneuro {

SmrBci::SmrBci(void) : p_nh_("~") {
	this->sub_topic_data_	 = "/neurodata";
	this->pub_topic_data_	=  "neuroprediction";

       	this->buffer_ = new wtk::proc::RingBuffer();
	this->laplacian_ = new wtk::proc::Laplacian();
	this->psd_ = new wtk::proc::Pwelch();
	
}

SmrBci::~SmrBci(void) {

}

bool SmrBci::configure(void) {

	ros::param::param("~buffer_size", (int&) this->buffer_size_, 512);
	ros::param::param("~n_channels", (int&) this->n_channels_, 16);
	ros::param::param("~sampling_freq", (int&) this->sampling_freq_, 512);
	ros::param::param("~n_samples", (int&) this->n_samples_, 32);

	if(ros::param::get("~lap_path", this->lap_path_) == false) {
		this->lap_path_ = "/home/whiteam/.whitk/data/lapmask_16ch_eog12_16.dat";
	}


	ros::param::param("~psd_wlength", (int&) this->psd_wlength_, 256);
	ros::param::param("~psd_novl", (int&) this->psd_novl_, 128);
	ros::param::param("~psd_dolog", (int&) this->psd_dolog_, 1); 


	ros::param::param<std::string>("~decoder_type", this->decoder_type_, "Gaussian");

	if(ros::param::get("~decoder_path", this->decoder_path_) == false) {
		ROS_ERROR("Missing 'decoder_path' in the SmrBci. 'decoder_path' is a mandatory parameter");
		return false;
	}


	ros::param::param("~n_classes", (int&) this->n_classes_, 2);

	// Setup Ring buffer
	this->buffer_->Setup(this->buffer_size_, this->n_channels_);

	// Setup Laplacian filter
	try {
		this->laplacian_->Setup(this->lap_path_);
	} catch (std::runtime_error& e) {
		throw std::runtime_error("Can't setup Laplacian filter because: '" + std::string(e.what()) + "'");
	}

	
	// Setup Pwelch method
	this->psd_->Setup(this->psd_wlength_, 
			  wtk::proc::Window::AsHamming, 
			  this->psd_novl_, 
			  this->sampling_freq_, 
			  this->psd_dolog_);

	
	// Setup decoder
	// Gloria Beraldo: TO DO generalize with different kind of decoder

	if(this->decoder_type_.std::string::compare("Gaussian")==0) {

		this->decoder_ = new wtk::proc::Gaussian();
		try {
			this->decoder_->Setup(this->decoder_path_);
		} catch (std::runtime_error& e) {
			throw std::runtime_error("Can't setup decoder because: '" + std::string(e.what()) + "'");
		}
	}

	
	// Setup temporary data matrices
	this->dmap_ = Eigen::MatrixXf::Zero(this->n_channels_, this->n_samples_);
	this->dbuf_ = Eigen::MatrixXd::Zero(this->buffer_size_, this->n_channels_);
	this->dlap_ = Eigen::MatrixXd::Zero(this->buffer_size_, this->n_channels_);
	this->dfet_ = Eigen::VectorXd::Zero(this->decoder_->config.nfeatures);

	this->rawpp_ = Eigen::VectorXd::Zero(this->decoder_->config.nclasses);
	//this->intpp_ = Eigen::VectorXd::Zero(this->decoder_->config.nclasses); 


	// Created by L.Tonin  <luca.tonin@dei.unipd.it> on 28/06/23 13:35:16	
	for(auto it = this->decoder_->config.classlbs.begin(); it != this->decoder_->config.classlbs.end(); ++it)
		this->classes_.push_back(static_cast<int>(*it));
	
	if(this->classes_.size() != this->n_classes_) {
		ROS_ERROR("Number of classes in the classifier is different from the provided number of class");
		return false;
	}


	this->sub_data_ = this->p_nh_.subscribe(this->sub_topic_data_, 1000, &SmrBci::on_received_data, this);
	this->pub_data_ = this->p_nh_.advertise<rosneuro_msgs::NeuroOutput>(this->pub_topic_data_, 1);
	
	this->srv_classify_ = this->p_nh_.advertiseService("classify", &SmrBci::on_request_classify, this);
	this->srv_reset_   =  this->p_nh_.advertiseService("reset",  &SmrBci::on_request_reset, this);


	this->new_neuro_frame_ = false;


	//TO DO Create a function in NeuroDataTools see below
	this->msg_.decoder.classes = this->classes_;
	this->msg_.decoder.type    = this->decoder_type_;
	this->msg_.decoder.path    = this->decoder_path_;

	return true;
}



float SmrBci::GetFrameRate(void) {
	
	float framerate = 1000.0f*this->n_samples_/this->sampling_freq_;
	return framerate;
}


void SmrBci::Reset(void) {
	ROS_INFO("Reset Probabilities");

	this->msg_.header.stamp = ros::Time::now();
	this->msg_.softpredict.data = std::vector<float>(this->rawpp_.data(), this->rawpp_.data() + this->rawpp_.rows() * this->rawpp_.cols());
	this->pub_data_.publish(this->msg_);

}


void SmrBci::on_received_data(const rosneuro_msgs::NeuroFrame::ConstPtr& msg) {


	if(msg->eeg.info.nsamples == this->n_samples_ && msg->eeg.info.nchannels == this->n_channels_) {
		this->new_neuro_frame_ = true;
		this->data_ = msg->eeg.data;
		this->msg_.neuroheader = msg->neuroheader;
	}

}


bool SmrBci::Classify(void) {

	// Copy data in eigen structure
	if(this->new_neuro_frame_== false)
	{
		//ROS_WARN("Not available data to classify");
		return false;
	}


	this->dmap_ = Eigen::Map<Eigen::MatrixXf>(this->data_.data(), this->n_channels_, this->n_samples_);
	this->dmap_.transposeInPlace();

	this->buffer_->Add(this->dmap_.cast<double>());
	
	if(this->buffer_->IsFull() == false)
	{
		//ROS_WARN("The buffer is not full");
		return false;
	}

	this->buffer_->Get(this->dbuf_);

	this->laplacian_->Apply(this->dbuf_, this->dlap_);

	this->psd_->Apply(this->dlap_);
	this->psd_->Get(this->dfet_, this->decoder_->config.idchan, this->decoder_->config.idfreq);

	this->decoder_->Run(this->dfet_, this->rawpp_);


	//publish msg with raw prob
		
	this->msg_.header.stamp = ros::Time::now();
	this->msg_.softpredict.data = std::vector<float>(this->rawpp_.data(), this->rawpp_.data() + this->rawpp_.rows() * this->rawpp_.cols());


	this->rawpp_.maxCoeff(&this->predicted_class_);

	this->hard_prediction_ = std::vector<int> (this->n_classes_);
	this->hard_prediction_.at(this->predicted_class_) = 1;

	this->msg_.hardpredict.data = this->hard_prediction_;
	this->pub_data_.publish(this->msg_);


	this->new_neuro_frame_= false;
	

	return true;

}


bool SmrBci::on_request_classify(std_srvs::Empty::Request& req,
								 std_srvs::Empty::Response& res) {
	return SmrBci::Classify();
}


bool SmrBci::on_request_reset(std_srvs::Empty::Request& req,
								 std_srvs::Empty::Response& res) {
	SmrBci::Reset();
	return true;
}



}


#endif
