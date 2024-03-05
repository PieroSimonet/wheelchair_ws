#ifndef CV_EMAPC_CPP
#define CV_EMAPC_CPP

#include <ema_pc/ema_pc.h>


using namespace ema_pc;

EmaPc::EmaPc(ros::NodeHandle& nh) {
  this->nh_ = nh;
  this->new_data_ = false;
  this->first_run_ = true;

  this->new_cloud_  = PointCloud();
  this->mean_cloud_ = PointCloud();

  this->ema_coeff_ = 0.1;
  this->hz_ = 16;
  
  this->nh_.param("ema_coeff", this->ema_coeff_, this->ema_coeff_);
  this->nh_.param("hz", this->hz_, this->hz_);

  this->sub_ = nh_.subscribe("pc_input", 1, &EmaPc::on_recived_poincloud, this);
  this->pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("pc_output", 1);
}

void EmaPc::run() {
  ros::Rate loop_rate(this->hz_);
  while (ros::ok()) {
    ros::spinOnce();
    if (this->new_data_) {
      this->process_data();
      this->new_data_ = false;
      this->pub_.publish(this->mean_cloud_);
    }
    loop_rate.sleep();
  }
}

void EmaPc::on_recived_poincloud(const PointCloud::ConstPtr& msg) {
  new_cloud_ = *msg;
  this->new_data_ = true;
}

void EmaPc::process_data() { 
  if (this->first_run_) {
    this->mean_cloud_ = this->new_cloud_;
    this->first_run_ = false;
  } else {
	  // AVERAGING OF THE POINTCLOUD----------------------------------------
	  for(int j=0;j<mean_cloud_.height;j++){
	  	for(int i=0;i<mean_cloud_.width;i++){
	  		// POINTCLOUD AVERAGING UPDATE
        // TODO: use the eigen library
	  		//if(new_cloud_.at(i,j).x!=0.0f && new_cloud_.at(i,j).y!=0.0f) {
	  			// EXPONENTIAL MOVING AVERAGE (CAN BE MADE MORE RESPONSIVE BY LOWERING PARAMETER "ema_coeff")
	  			mean_cloud_.at(i,j).x = ema_coeff_*mean_cloud_.at(i,j).x + (1-ema_coeff_)*new_cloud_.at(i,j).x;
	  			mean_cloud_.at(i,j).y = ema_coeff_*mean_cloud_.at(i,j).y + (1-ema_coeff_)*new_cloud_.at(i,j).y;
	  			mean_cloud_.at(i,j).z = ema_coeff_*mean_cloud_.at(i,j).z + (1-ema_coeff_)*new_cloud_.at(i,j).z;
 	  		//}
     	}
	  }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ema_pc");
  ros::NodeHandle nh;
  EmaPc ema_pc(nh);
  ema_pc.run();
  return 0;
}


#endif
