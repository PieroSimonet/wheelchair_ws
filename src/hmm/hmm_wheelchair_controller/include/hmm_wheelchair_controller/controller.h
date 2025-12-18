#ifndef ROSNEURO_CONTROLLER_HMM_H_
#define ROSNEURO_CONTROLLER_HMM_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include "rosneuro_msgs/NeuroOutput.h"
#include "rosneuro_msgs/NeuroEvent.h"
#include <string>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <gazebo_msgs/GetModelState.h>
#include <rosgraph_msgs/Clock.h>
#include <std_srvs/Empty.h>
#include <cmath>
#include <ctime>
#include <fstream>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace rosneuro {

struct {
  int init = 1;

  int continous_feedback = 781;

  int rest = 783; // C
  int bh   = 773; // left
  int bf   = 771; // rigth
  
  int fixation = 786;

  int hit  = 897;
  int miss = 898;

  int start_mv = 1800;
  int end_mv   = 1900;
  int error_rq = 5000;

  int close_msk = 32768;
} static constexpr EVENTS_ID;


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
        command get_command();
        void check_probability();

        void callback_probability(const rosneuro_msgs::NeuroOutput::ConstPtr& msg);
        void callback_laser(const sensor_msgs::LaserScan::ConstPtr& msg);
        void callback_odom(const nav_msgs::Odometry::ConstPtr& msg);
        void callback_events(const rosneuro_msgs::NeuroEvent::ConstPtr& msg);

        void callback_goalreach(const std_msgs::Bool::ConstPtr& msg);

        void cb_smr_raw(const rosneuro_msgs::NeuroOutput::ConstPtr& msg);
        void cb_smr_integrated(const rosneuro_msgs::NeuroOutput::ConstPtr& msg);
        void cb_hmm_raw(const rosneuro_msgs::NeuroOutput::ConstPtr& msg);

        geometry_msgs::Twist generate_command();
        void publish_command(geometry_msgs::Twist cmd);

        bool is_gazebo_ready();
        void is_goal_reached();

        void end_fixation_callback(const ros::TimerEvent& ev);
        void cue_not_reached_callback(const ros::TimerEvent& ev);
        void end_visual_cue_callback(const ros::TimerEvent& ev);

        void require_reset_integration();

        void check_subgoal();
        void check_goal();
        void close_procedure();
        void generate_sub_goal();

        void generate_navgoal();
        void check_navgoal();

        void init_save_file();
        void update_save_file();

        void send_event(int ev);

    private:
        // Thresholds to cross to send a command
        float threshold_l_;
        float threshold_c_;
        float threshold_r_;

        command current_command_;

        bool goal_reached_;
        bool threshold_reached_;
        bool is_robot_moving_;
        bool is_waiting_for_joyevent_;

        // I need to check if I need to wait for fixation
        bool is_in_fixation_gui_;

        bool has_prob_;

        ros::NodeHandle nh_;

        ros::Subscriber sub_probability_;
        ros::Subscriber sub_laser_;
        ros::Subscriber sub_odom_;
        ros::Subscriber sub_nav_stop_;
        ros::Subscriber sub_events_;

        // Subscriber on different values
        ros::Subscriber sub_smr_raw_, sub_smr_integrated_, sub_hmm_raw_;

        ros::ServiceClient srv_reset_integration_;
        ros::ServiceClient srv_reset_integration_hmm_;
        ros::ServiceClient srv_reset_hmm_;

        ros::Publisher pub_cmd_;
        ros::Publisher pub_evs_;
        ros::Publisher pub_fake_joy_;

        ros::Timer fixation_callback_timer_;
        ros::Timer cue_not_reached_timer_;
        ros::Timer visual_cue_timer_;

        ros::ServiceClient gazebo_get_model_state_;

        int rate_freq_;
        // ros::Rate loop_rate_;

        // Current state
        nav_msgs::Odometry         current_odom_ = nav_msgs::Odometry();
        sensor_msgs::LaserScan     current_laser_;
        rosneuro_msgs::NeuroOutput current_prob_ = rosneuro_msgs::NeuroOutput();

        sensor_msgs::Joy fake_joy_ = sensor_msgs::Joy();

        // Additional information
        rosneuro_msgs::NeuroOutput smr_raw_ = rosneuro_msgs::NeuroOutput();
        rosneuro_msgs::NeuroOutput smr_integrated_ = rosneuro_msgs::NeuroOutput();
        rosneuro_msgs::NeuroOutput hmm_raw_ = rosneuro_msgs::NeuroOutput();

        // Event msg
        rosneuro_msgs::NeuroEvent msg_ev_;

        int num_commands_    = 0;
        int num_reject_cmds_ = 0;

        int requested_cue_ = 0;

        // Subgoal
        geometry_msgs::Pose subgoal_;

        // Internal PID
        struct i_pid {
            float prev_error = 0.0f;
            float output_max = 1.0f;
            float integral   = 0.0f;
            float derivate   = 0.0f;

            float output     = 0.0f;

            float dt = 1.0/50.0f;

            float P = 1.0f;
            float D = 0.5f;
            float I = 0.0f;  // I try only with pi now

            void reset() {
              prev_error = 0.0f;
              integral   = 0.0f;
            }

            float compute(float new_error) {
                integral += new_error;
                derivate = (dt > 0.0) ? (new_error - prev_error) / dt : 0.0;

                output = P * new_error + I * integral + D * derivate;

                if (std::abs(output) > output_max)
                    output = std::copysign(output_max, output);

                prev_error = new_error;

                return output;
            }

        } pid_x_, pid_w_;

        // Final position
        nav_msgs::Odometry final_odom_;

        // csv file to save the information
        std::string csv_file_;

        std::ofstream file_;

        // Information of the run
        std::string sub_name_;
        ros::Time start_time_;
        ros::Time end_time_;
        std::string file_name_;
        std::string file_path_;
        std::string task_; // simulation or real device
        bool is_real_ = false;
        
        int last_event_;

        bool is_sub_goalreached = false;
        bool is_already_stopped = true;

};

} // namespace rosneuro

#endif
