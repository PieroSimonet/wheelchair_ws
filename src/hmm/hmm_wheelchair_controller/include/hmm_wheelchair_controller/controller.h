#ifndef ROSNEURO_CONTROLLER_HMM_H_
#define ROSNEURO_CONTROLLER_HMM_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "rosneuro_msgs/NeuroOutput.h"
#include <string>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/GetModelState.h>
#include <rosgraph_msgs/Clock.h>
#include <std_srvs/Empty.h>
#include <cmath>

namespace rosneuro {

class controller {
    public:
    enum class command {LEFT, CENTER, RIGHT};

    public:
        controller(void);
        ~controller(void);

        bool configure(void);
        void run(void);

    protected:

    private:
        bool is_command_accetable(command cmd);
        void send_command(geometry_msgs::Twist cmd);
        void save_csv();
        command get_command();
        void check_probability();

        void callback_probability(const rosneuro_msgs::NeuroOutput::ConstPtr& msg);
        void callback_laser(const sensor_msgs::LaserScan::ConstPtr& msg);
        void callback_odom(const nav_msgs::Odometry::ConstPtr& msg);

        geometry_msgs::Twist generate_command();
        void publish_command(geometry_msgs::Twist cmd);

        bool is_gazebo_ready();
        void is_goal_reached();

        void require_reset_integration();

        void check_goal();
        void close_procedure();

    private:
        // Thresholds to cross to send a command
        float threshold_l_;
        float threshold_c_;
        float threshold_r_;

        command current_command_;

        bool goal_reached_;
        bool threshold_reached_;

        bool has_prob_;

        ros::NodeHandle nh_;

        ros::Subscriber sub_probability_;
        ros::Subscriber sub_laser_;
        ros::Subscriber sub_odom_;

        ros::ServiceClient srv_reset_integration_;

        ros::Publisher pub_cmd_;

        ros::ServiceClient gazebo_get_model_state_;

        int rate_freq_;
        // ros::Rate loop_rate_;

        // Current state
        nav_msgs::Odometry         current_odom_;
        sensor_msgs::LaserScan     current_laser_;
        rosneuro_msgs::NeuroOutput current_prob_;

        // Final position
        nav_msgs::Odometry final_odom_;

        // csv file to save the information
        std::string csv_path_;
        //std::ofstream file_;

        // Information of the run
        std::string sub_name_;
        ros::Time start_time_;
        ros::Time end_time_;
        std::string file_name_;
        std::string file_path_;
        std::string modality_;

};

} // namespace rosneuro

#endif
