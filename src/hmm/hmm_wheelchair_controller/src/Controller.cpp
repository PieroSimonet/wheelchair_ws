#ifndef ROSNEURO_CONTROLLER_HMM_CPP
#define ROSNEURO_CONTROLLER_HMM_CPP

#include "hmm_wheelchair_controller/controller.h"

namespace rosneuro {

std::vector<float> string2vector_converter(std::string msg){
    // If possible, always prefer std::vector to naked array
    std::vector<float> v;

    // Build an istream that holds the input string
    std::istringstream iss(msg);

    // Iterate over the istream, using >> to grab floats
    // and push_back to store them in the vector
    std::copy(std::istream_iterator<float>(iss),
    std::istream_iterator<float>(),
    std::back_inserter(v));

    return v;
}

controller::controller() {
    std::string tmp_t = "0.9 0.9 0.9";
    
    ros::param::param<std::string>("~thresholds", tmp_t, tmp_t);
    std::vector<float> thresholds_tmp = string2vector_converter(tmp_t);

    threshold_l_ = thresholds_tmp[0];
    threshold_c_ = thresholds_tmp[1];
    threshold_r_ = thresholds_tmp[2];

    ros::param::param<int>("~loop_rate", this->rate_freq_, 16);
    ros::param::param<std::string>("~csv_path", this->csv_path_, "/tmp/hmm_wheelchair_controller.csv");

    this->goal_reached_ = false;
    this->threshold_reached_ = false;
    this->has_prob_ = false;

    this->final_odom_ = nav_msgs::Odometry();
    this->final_odom_.pose.pose.position.x = 15.0;
    this->final_odom_.pose.pose.position.y = 1.3;
}

controller::~controller() {}

void controller::callback_probability(const rosneuro_msgs::NeuroOutput::ConstPtr& msg) {
    this->current_prob_ = *msg;
    this->has_prob_ = true;
}

void controller::callback_laser(const sensor_msgs::LaserScan::ConstPtr& msg) {
    this->current_laser_ = *msg;
}

void controller::callback_odom(const nav_msgs::Odometry::ConstPtr& msg) {
    this->current_odom_ = *msg;
}

bool controller::is_gazebo_ready() {
    //wait for the service and ask the results
    //ros::service::waitForService("/gazebo/get_model_state");
    //return ros::service::exists("/gazebo/get_model_state", true);
    // Also this is not working, need to attach to keyboard related event
    auto message = ros::topic::waitForMessage<rosgraph_msgs::Clock>("/clock", this->nh_, ros::Duration(100));
    return true;
}

bool controller::configure() {

    // Setup listeners
    this->sub_probability_ = nh_.subscribe("/hmm/neuroprediction/integrated", 1, &controller::callback_probability, this);
    this->sub_laser_       = nh_.subscribe("/fused_scan", 1, &controller::callback_laser, this);
    this->sub_odom_        = nh_.subscribe("/odometry/filtered", 1, &controller::callback_odom, this);

    // Setup services
    this->srv_reset_integration_ = nh_.serviceClient<std_srvs::Empty>("/integrator/reset");

    // Setup publishers
    this->pub_cmd_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    while(!this->is_gazebo_ready()) {
        ros::Duration(0.1).sleep();
    }

    ROS_INFO("Gazebo is ready, start recoding");

    this->start_time_ = ros::Time::now(); // TODO: use a topic to get the start time gained by pressing a button

    return true;
}

void controller::check_goal() {
    float dx = this->current_odom_.pose.pose.position.x - this->final_odom_.pose.pose.position.x;
    float dy = this->current_odom_.pose.pose.position.y - this->final_odom_.pose.pose.position.y;
    float dist = sqrt(dx*dx + dy*dy);
    if (dist < 1.0) { // The goal is reached if I am less than 1 meter away
        this->goal_reached_ = true;
    }
}

void controller::require_reset_integration() {
    std_srvs::Empty emp;
    this->srv_reset_integration_.call(emp);
}

void controller::check_probability() {
    std::vector<float> probs = this->current_prob_.softpredict.data;

    if (probs[0] > this->threshold_l_) {
        this->threshold_reached_ = true;
        this->current_command_ = controller::command::LEFT;
    } else if (probs[1] > this->threshold_c_) {
        this->threshold_reached_ = true;
        this->current_command_ = controller::command::CENTER;
    } else if (probs[2] > this->threshold_r_) {
        this->threshold_reached_ = true;
        this->current_command_ = controller::command::RIGHT;
    }
}

geometry_msgs::Twist controller::generate_command() {
    geometry_msgs::Twist cmd;

    if (this->is_command_accetable(this->current_command_)) {
        switch (this->current_command_) {
            case controller::command::LEFT:
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.5;
                break;
            case controller::command::CENTER:
                cmd.linear.x = 0.5;
                cmd.angular.z = 0.0;
                break;
            case controller::command::RIGHT:
                cmd.linear.x = 0.0;
                cmd.angular.z = -0.5;
                break;
          
        }
    }
    return cmd;
}

void controller::run() {
    ros::Rate loop_rate(this->rate_freq_);

    while (ros::ok() && !this->goal_reached_) {
        if (this->has_prob_) {

            this->check_probability();

            if (this->threshold_reached_) {
                this->send_command(this->generate_command());
                this->threshold_reached_ = false;
                this->require_reset_integration();
            }

            this->check_goal();
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    this->close_procedure();

}

bool controller::is_command_accetable(controller::command cmd) {
    // Check the position in the scan message if the laser is too close in the requested direction
    return true;
}

void controller::send_command(geometry_msgs::Twist cmd) {}

void controller::save_csv() {}

void controller::close_procedure() {}

}

#endif
