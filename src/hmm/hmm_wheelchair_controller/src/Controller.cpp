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

    ros::param::param<int>("~loop_rate", this->rate_freq_, 50);
    ros::param::param<std::string>("~csv_path", this->file_path_, "/tmp/");
    ros::param::param<std::string>("~subject",  this->sub_name_, "test" );
    ros::param::param<std::string>("~task",     this->task_, "real");

    // Check if the command is given
    this->is_real_ = false; (this->task_.compare("real") == 0);

    /*if (this->is_real_){
      ROS_INFO("Real device");
    }else {
      ROS_INFO("Simulation");
    }*/

    this->goal_reached_ = false;
    this->threshold_reached_ = false;
    this->has_prob_ = false;
    this->is_robot_moving_ = false;
    this->is_in_fixation_gui_ = false;
    this->is_waiting_for_joyevent_ = false;

    this->final_odom_ = nav_msgs::Odometry();
    this->final_odom_.pose.pose.position.x = 15.0;
    this->final_odom_.pose.pose.position.y = 1.3;

    this->last_event_ = 0;

    this->msg_ev_ = rosneuro_msgs::NeuroEvent();
    this->msg_ev_.header.frame_id = "controller";

    this->smr_raw_.softpredict.data = {0.0f, 0.0f};
    this->smr_integrated_.softpredict.data = {0.0f, 0.0f};
    this->hmm_raw_.softpredict.data = {0.0f, 0.0f, 0.0f};
    this->current_prob_.softpredict.data = {0.0f, 0.0f, 0.0f};

}

controller::~controller() {}

void controller::callback_probability(const rosneuro_msgs::NeuroOutput::ConstPtr& msg) {
    this->current_prob_ = *msg;
    this->has_prob_ = true;
}


// ------ This is just to save the data
void controller::cb_smr_raw(const rosneuro_msgs::NeuroOutput::ConstPtr& msg) {
    this->smr_raw_ = *msg;
}

void controller::cb_smr_integrated(const rosneuro_msgs::NeuroOutput::ConstPtr& msg) {
    this->smr_integrated_ = *msg;
}

void controller::cb_hmm_raw(const rosneuro_msgs::NeuroOutput::ConstPtr& msg) {
    this->hmm_raw_ = *msg;
}
// ------

void controller::callback_laser(const sensor_msgs::LaserScan::ConstPtr& msg) {
    this->current_laser_ = *msg;
}

void controller::callback_odom(const nav_msgs::Odometry::ConstPtr& msg) {
    this->current_odom_ = *msg;
}

void controller::callback_events(const rosneuro_msgs::NeuroEvent::ConstPtr& msg) {
    // I need this only in the case "real"

    if(this->task_.compare("real") != 0){
        this->is_waiting_for_joyevent_ = false;
    }

    if (this->is_waiting_for_joyevent_) {
        switch (msg->event) {
          case 2783:
          case 2773:
          case 2771:
            this->is_waiting_for_joyevent_ = false;
            this->requested_cue_ = msg->event - 2000;
            break;
        } 
    }
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
    this->sub_probability_ = nh_.subscribe("/hmm/neuroprediction/integrated", 10, &controller::callback_probability, this);
    this->sub_laser_       = nh_.subscribe("/fused_scan", 1, &controller::callback_laser, this);
    this->sub_odom_        = nh_.subscribe("/odometry/filtered", 1, &controller::callback_odom, this);
    this->sub_nav_stop_    = nh_.subscribe("/goalreached", 1, &controller::callback_goalreach, this);

    // Setup additional listeners
    this->sub_smr_raw_        = nh_.subscribe("/smrbci/neuroprediction", 1, &controller::cb_smr_raw, this);
    this->sub_smr_integrated_ = nh_.subscribe("/smr/neuroprediction/integrated", 1, &controller::cb_smr_integrated, this);
    this->sub_hmm_raw_        = nh_.subscribe("/hmm/neuroprediction", 1, &controller::cb_hmm_raw, this);
    this->sub_events_         = nh_.subscribe("/events/bus", 10, &controller::callback_events, this);

    // Setup services
    this->srv_reset_integration_     = nh_.serviceClient<std_srvs::Empty>("/integrator/reset");
    this->srv_reset_integration_hmm_ = nh_.serviceClient<std_srvs::Empty>("/integrator_hmm/reset");
    this->srv_reset_hmm_             = nh_.serviceClient<std_srvs::Empty>("/hmm/reset");

    // Setup publishers
    this->pub_cmd_ = nh_.advertise<geometry_msgs::Twist>("/gui/cmd_vel", 1);
    this->pub_evs_ = nh_.advertise<rosneuro_msgs::NeuroEvent>("/events/bus", 1);
    this->pub_fake_joy_ = nh_.advertise<sensor_msgs::Joy>("/joy", 1);

    if (this->task_.compare("real") == 1) {
        while(!this->is_gazebo_ready()) {
            ros::Duration(0.1).sleep();
        }
    }

    ROS_INFO("Gazebo is ready, start recoding");

    this->start_time_ = ros::Time::now(); // TODO: use a topic to get the start time gained by pressing a button

    return true;
}

void controller::send_event(int ev) {

    auto send_event_ = [this](int ev) {
        this->msg_ev_.header.stamp = ros::Time::now();
        this->msg_ev_.event = ev;
        this->pub_evs_.publish(this->msg_ev_);
    };

    send_event_(ev);

    // Also save this information for the csv file
    this->last_event_ = ev;

    ros::Duration(0.1).sleep();
    send_event_(ev + EVENTS_ID.close_msk);

}

void controller::check_goal() {
    float dx = this->current_odom_.pose.pose.position.x - this->final_odom_.pose.pose.position.x;
    float dy = this->current_odom_.pose.pose.position.y - this->final_odom_.pose.pose.position.y;
    float dist = sqrt(dx*dx + dy*dy);
    if (dist < 3.0) { // The goal is reached if I am less than 1 meter away
        this->goal_reached_ = true;
    }
}

void controller::require_reset_integration() {
    this->current_prob_.softpredict.data = {0.0,0.0,0.0};
    std_srvs::Empty emp;
    this->srv_reset_integration_.call(emp);
    this->srv_reset_integration_hmm_.call(emp);
}

void controller::check_probability() {
    std::vector<float> probs = this->current_prob_.softpredict.data;

    if (probs[0] > this->threshold_l_) {
        this->threshold_reached_ = true;
        ROS_INFO("Left threshold reached");
        this->current_command_ = controller::command::LEFT;
        this->send_event(EVENTS_ID.bh);
    } else if (probs[1] > this->threshold_c_) {
        this->threshold_reached_ = true;
        ROS_INFO("Center threshold reached");
        this->send_event(EVENTS_ID.rest);
        this->current_command_ = controller::command::CENTER;
    } else if (probs[2] > this->threshold_r_) {
        this->threshold_reached_ = true;
        ROS_INFO("Right threshold reached");
        this->current_command_ = controller::command::RIGHT;
        this->send_event(EVENTS_ID.bf);
    }

    if (this->threshold_reached_) {
      ros::Duration(0.1).sleep();
      this->send_event(EVENTS_ID.hit);
    }
}

void controller::callback_goalreach(const std_msgs::Bool::ConstPtr& msg) {
    this->is_sub_goalreached = msg->data;
}

void controller::check_subgoal() {

    double roll, pitch, yaw, current_yaw;

    tf2::Quaternion q(
        this->subgoal_.orientation.x,
        this->subgoal_.orientation.y,
        this->subgoal_.orientation.z,
        this->subgoal_.orientation.w
    );

    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    tf2::Quaternion q_c(
        this->current_odom_.pose.pose.orientation.x,
        this->current_odom_.pose.pose.orientation.y,
        this->current_odom_.pose.pose.orientation.z,
        this->current_odom_.pose.pose.orientation.w
    );

    tf2::Matrix3x3(q_c).getRPY(roll, pitch, current_yaw);

    double error_d = yaw-current_yaw;

    float dx = this->current_odom_.pose.pose.position.x - this->subgoal_.position.x;
    float dy = this->current_odom_.pose.pose.position.y - this->subgoal_.position.y;
    float dist = sqrt(dx*dx + dy*dy);

    bool reached = true;

    switch (this->current_command_) {
        case controller::command::LEFT:
        case controller::command::RIGHT:
            if (std::abs(error_d) > 0.02) {
                reached = false;
            }
            break;
        case controller::command::CENTER:
            if (std::abs(dist) > 1.0) {
                reached = false;
            }
            break;
    }

    if (reached) {
        // Stop the robot and proceed to the next command
        this->send_command(geometry_msgs::Twist());
        this->is_robot_moving_ = false;
        this->require_reset_integration();
        this->pid_w_.reset();
        this->pid_x_.reset();

        //this->subgoal_.position.x = this->current_odom_.pose.pose.position.x;
        //this->subgoal_.position.y = this->current_odom_.pose.pose.position.y;

        // Set the gui as a fixation
        this->is_in_fixation_gui_ = true;
        this->is_waiting_for_joyevent_ = true;
        this->send_event(EVENTS_ID.fixation);
        this->fixation_callback_timer_ = this->nh_.createTimer(ros::Duration(3.0), &controller::end_fixation_callback, this, true);
    }
}

void controller::check_navgoal() {

    if (this->is_sub_goalreached) {
        if (!this->is_already_stopped) {
            this->is_already_stopped = true;
        } else {
          return;
        }
    }else {
        return;
    }

    // Stop the robot and proceed to the next command
    this->send_command(geometry_msgs::Twist());
    this->is_robot_moving_ = false;
    this->require_reset_integration();
    this->pid_w_.reset();
    this->pid_x_.reset();

    // Set the gui as a fixation
    this->is_in_fixation_gui_ = true;
    this->is_waiting_for_joyevent_ = true;
    this->send_event(EVENTS_ID.fixation);
    this->require_reset_integration();
    this->fixation_callback_timer_ = this->nh_.createTimer(ros::Duration(3.0), &controller::end_fixation_callback, this, true);

}

void controller::end_fixation_callback(const ros::TimerEvent& ev) {

    //ROS_ERROR("------------ ");

    if (this->is_waiting_for_joyevent_) {
        //ROS_ERROR("AAAAAA ");
        // Wait for the user joy
        this->fixation_callback_timer_ = this->nh_.createTimer(ros::Duration(1.0), &controller::end_fixation_callback, this, true);
    } else {
        //ROS_ERROR("BBBBB ");
        // PRINT CUE 
        this->send_event(this->requested_cue_);
        this->visual_cue_timer_ = this->nh_.createTimer(ros::Duration(2.0), &controller::end_visual_cue_callback, this, true);
    }
}

void controller::end_visual_cue_callback(const ros::TimerEvent& ev) {
    this->require_reset_integration();
    this->is_in_fixation_gui_ = false;
    this->send_event(EVENTS_ID.continous_feedback);
    // Launch the timer callback for the threshold not reached
    this->cue_not_reached_timer_ = this->nh_.createTimer(ros::Duration(6.0), &controller::cue_not_reached_callback, this, true);
}


void controller::cue_not_reached_callback(const ros::TimerEvent& ev) {
    if (!this->threshold_reached_) {
        // Ensure that it will not do the other control
        this->is_in_fixation_gui_ = true;
        this->send_event(EVENTS_ID.miss);
       // Set the event in the save file
       this->update_save_file();
       // Retry the cue from the fixation
       this->require_reset_integration();
       this->send_event(EVENTS_ID.fixation);
       this->fixation_callback_timer_ = this->nh_.createTimer(ros::Duration(3.0), &controller::end_fixation_callback, this, true);
    }
}


geometry_msgs::Twist controller::generate_command() {

    double roll, pitch, yaw, current_yaw;

    tf2::Quaternion q(
        this->subgoal_.orientation.x,
    this->subgoal_.orientation.y,
        this->subgoal_.orientation.z,
        this->subgoal_.orientation.w
    );

    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    tf2::Quaternion q_c(
        this->current_odom_.pose.pose.orientation.x,
        this->current_odom_.pose.pose.orientation.y,
        this->current_odom_.pose.pose.orientation.z,
        this->current_odom_.pose.pose.orientation.w
    );

    tf2::Matrix3x3(q_c).getRPY(roll, pitch, current_yaw);

    double error_d = yaw-current_yaw;

    float dx = this->current_odom_.pose.pose.position.x - this->subgoal_.position.x;
    float dy = this->current_odom_.pose.pose.position.y - this->subgoal_.position.y;
    float dist = sqrt(dx*dx + dy*dy);

    ROS_INFO("------------ %f", dist);

    geometry_msgs::Twist cmd;

    switch (this->current_command_) {
        case controller::command::CENTER:
            cmd.linear.x = this->pid_x_.compute(dist);
        case controller::command::LEFT:
        case controller::command::RIGHT:
            cmd.angular.z = this->pid_w_.compute(error_d);
            break;
    }

    return cmd;
}


void controller::generate_sub_goal() {
    this->subgoal_ = this->current_odom_.pose.pose;

    tf2::Quaternion q(
      this->subgoal_.orientation.x,
      this->subgoal_.orientation.y,
      this->subgoal_.orientation.z,
      this->subgoal_.orientation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    float delta_d = 6.0f; 
    float sub_yaw = M_PI/2.0f;

    if (this->task_.compare("real") == 0) {
        delta_d = 2.5f;
        sub_yaw = M_PI/2.0f;
    }

    if (this->is_command_accetable(this->current_command_)) {
        switch (this->current_command_) {
            case controller::command::LEFT:
                yaw += sub_yaw;
                break;
            case controller::command::CENTER:
                this->subgoal_.position.x += delta_d * std::cos(yaw);
                this->subgoal_.position.y += delta_d * std::sin(yaw);
                break;
            case controller::command::RIGHT:
                yaw -= sub_yaw;
                break;
          
        }
    }else {
        this->num_reject_cmds_++;
    }

    q.setRPY(roll, pitch, yaw);

    this->subgoal_.orientation.x = q.x();
    this->subgoal_.orientation.y = q.y();
    this->subgoal_.orientation.z = q.z();
    this->subgoal_.orientation.w = q.w();

}

void controller::generate_navgoal() {

    if (this->is_command_accetable(this->current_command_)) {
        this->fake_joy_ = sensor_msgs::Joy();
        this->fake_joy_.header.stamp = ros::Time::now();
        this->fake_joy_.axes    = {0,0,0,0,0,0,0,0};
        this->fake_joy_.buttons = {0,0,0,0,0,0,0,0};
        switch (this->current_command_) {
            case controller::command::LEFT:
                this->fake_joy_.axes[6] =  1;
                break;
            case controller::command::CENTER:
                this->fake_joy_.axes[7] =  1;
                break;
            case controller::command::RIGHT:
                this->fake_joy_.axes[6] = -1;
                break;
        }
        // Send the fake joy cmd
        this->pub_fake_joy_.publish(this->fake_joy_);
    }else {
        this->num_reject_cmds_++;
    }
}

void controller::run() {
    ros::Rate loop_rate(this->rate_freq_);

    this->init_save_file();
    this->send_event(EVENTS_ID.init);

     if (this->is_real_) {
        // The real devices
        this->check_navgoal();
    } else {
        // The simulation
        this->check_subgoal();
    }

    this->check_subgoal();


    while (ros::ok() && !this->goal_reached_) {
        if (this->is_robot_moving_) {
          // Here we need to divide two words simulation and real
          if (this->is_real_) {
              // The real devices
              this->check_navgoal();
          } else {
              // The simulation
              this->send_command(this->generate_command());
              this->check_subgoal();
          }
        }else if (this->has_prob_ && !this->is_in_fixation_gui_) {
            this->check_probability();

            if (this->threshold_reached_) {
                // Close the timer callback
                this->cue_not_reached_timer_.stop();

                if (this->is_real_) {
                    // Real devices 
                    this->generate_navgoal();
                    this->is_already_stopped = false;
                } else {
                    // Simulation device
                    this->generate_sub_goal();
                }
                this->is_robot_moving_ = true;
                this->threshold_reached_ = false;
                this->require_reset_integration();
                this->num_commands_++;
            }
            if (!this->is_real_) {
                this->send_command(geometry_msgs::Twist()); // Send "zero" cmd
            }
        }

        this->check_goal();

        this->update_save_file();

        loop_rate.sleep();
        ros::spinOnce();
    }

    ROS_WARN("----------- RUN ENDED ---------------");

    // Save the data
    this->close_procedure();

}

bool controller::is_command_accetable(controller::command cmd) {

    // TODO CHECK THE REQUESTED DIRECTION
    //if (this->is_real_)
    //    return true;

    if (cmd != controller::command::CENTER) 
        return true; // I only need to check if the wheelchair could go straight 

    // Check the position in the scan message if the laser is too close in the requested direction
    float angle_min = - M_PI/12.0f;
    float angle_max =   M_PI/12.0f;

    float min_distance = INFINITY;

    float current_angle = this->current_laser_.angle_min;
    float idx_angle = 0;

    while (current_angle < angle_max) {

        if (current_angle > angle_min && this->current_laser_.ranges[idx_angle] < min_distance) {
            min_distance = this->current_laser_.ranges[idx_angle];
        }

        current_angle += this->current_laser_.angle_increment;
        idx_angle++;
    }

    bool request = min_distance > 1.6f;
    
    if (!request) 
      this->send_event(EVENTS_ID.error_rq);

    return request;
}

void controller::send_command(geometry_msgs::Twist cmd) {
    this->pub_cmd_.publish(cmd);
}

void controller::init_save_file() {

    // Create the saving file
    std::time_t t = std::time(0);

    std::tm tm{};

    #ifdef _WIN32
        localtime_s(&tm, &t);
    #else
        localtime_r(&t, &tm);
    #endif

    std::ostringstream oss;

    oss << this->sub_name_ << "_" << std::put_time(&tm, "%Y%m%d_%H%M") << "_" << this->task_ << ".csv";

    this->file_name_ = oss.str();

    this->csv_file_ = this->file_path_ + "/" + this->file_name_;

    this->file_.open(this->csv_file_.c_str());

    // Now save the following information:
    // time, last_event, odom_x, odom_y, odom_w, raw_prob_0,raw_prob_1, int_prob_0, int_prob_1, hmm_raw_0, hmm_raw_1, hmm_raw_2, hmm_int_0, hmm_int_1, hmm_int_2, num_cmd, num_rej_cmd, completed

    std::string file_init = "time, last_event, odom_x, odom_y, odom_w, raw_prob_0,raw_prob_1, int_prob_0, int_prob_1, hmm_raw_0, hmm_raw_1, hmm_raw_2, hmm_int_0, hmm_int_1, hmm_int_2, num_cmd, num_rej_cmd, completed";

    this->file_ << file_init << std::endl;
}

void controller::update_save_file() {
    // time, last_event, odom_x, odom_y, odom_w, raw_prob_0,raw_prob_1, int_prob_0, int_prob_1, hmm_raw_0, hmm_raw_1, hmm_raw_2, hmm_int_0, hmm_int_1, hmm_int_2, num_cmd, num_rej_cmd

    tf2::Quaternion q(
      this->current_odom_.pose.pose.orientation.x,
      this->current_odom_.pose.pose.orientation.y,
      this->current_odom_.pose.pose.orientation.z,
      this->current_odom_.pose.pose.orientation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    std::string time = std::to_string(std::time(nullptr));

    std::string new_information = time + "," + std::to_string(this->last_event_) +","+ std::to_string(this->current_odom_.pose.pose.position.x) + "," + std::to_string(this->current_odom_.pose.pose.position.y) + "," + std::to_string(yaw) + ",";

    new_information += std::to_string(this->smr_raw_.softpredict.data[0]) + "," + std::to_string(this->smr_raw_.softpredict.data[1]) + "," + std::to_string(this->smr_integrated_.softpredict.data[0]) + "," + std::to_string(this->smr_integrated_.softpredict.data[1]) + ",";

    new_information += std::to_string(this->hmm_raw_.softpredict.data[0]) + "," + std::to_string(this->hmm_raw_.softpredict.data[1]) + "," + std::to_string(this->hmm_raw_.softpredict.data[2]) + ",";

    new_information += std::to_string(this->current_prob_.softpredict.data[0]) + "," + std::to_string(this->current_prob_.softpredict.data[1]) + "," + std::to_string(this->current_prob_.softpredict.data[2]) + ",";

    new_information += std::to_string(this->num_commands_) + "," + std::to_string(this->num_reject_cmds_) + "," + std::to_string(this->goal_reached_);

    this->file_ << new_information << std::endl;

    // ROS_INFO("Saved data: %s", new_information.c_str());
}

void controller::close_procedure() {
    this->update_save_file();
    // Check time
    // Save the csv and close all
    
    if (this->file_.is_open())
        this->file_.close();
}

}

#endif
