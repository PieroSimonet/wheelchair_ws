#ifndef ROSNEURO_FEEDBACK_BARS_H_
#define ROSNEURO_FEEDBACK_BARS_H_

#include <ros/ros.h>
#include <rosneuro_msgs/NeuroOutput.h>
#include <rosneuro_msgs/NeuroEvent.h>

#include "neurodraw/Engine.h"
#include "neurodraw/Palette.h"
#include "neurodraw/EventKey.h"
#include "neurodraw/Rectangle.h"
#include "neurodraw/Circle.h"
#include "neurodraw/Cross.h"
#include "neurodraw/Arrow.h"

namespace rosneuro {

class bars {
  public:
    enum class Task {
        LEFT = 0,
        RIGHT,
        CENTER,
        NONE
      };

    bars(void);
    virtual ~bars(void);

    bool configure();
    void run();

  protected:
    void setup_scene();
    void update();

    void on_received_neuro_data(const rosneuro_msgs::NeuroOutput& msg);
    void on_keyboard_event(const neurodraw::KeyboardEvent& event);

    void cb_events(const rosneuro_msgs::NeuroEvent::ConstPtr& msg);

    void clear_scene();
    void show_feedback();
    void show_fixation();
    void show_task(Task t);

    neurodraw::Engine* engine_;
    ros::NodeHandle nh_;

    bool user_quit_;

    std::vector<float> current_probablility_;

  private:
    ros::Subscriber sub_neuro_;
    ros::Subscriber sub_events_;

    neurodraw::Rectangle* probablility_l_;
    neurodraw::Rectangle* probablility_r_;
    neurodraw::Rectangle* probablility_c_;

    neurodraw::Rectangle* treshold_l_;
    neurodraw::Rectangle* treshold_r_;
    neurodraw::Rectangle* treshold_c_;

    neurodraw::Circle* circle_l_;
    neurodraw::Circle* circle_r_;
    neurodraw::Circle* circle_c_;

    // New Arrow
    neurodraw::Arrow* arrow_l_;
    neurodraw::Arrow* arrow_r_;
    neurodraw::Arrow* arrow_c_;

    neurodraw::Cross* fixation_cross_;

    neurodraw::Rectangle* bar_l_;
    neurodraw::Rectangle* bar_r_;
    neurodraw::Rectangle* bar_c_;

    neurodraw::Rectangle* cbar_l_;
    neurodraw::Rectangle* cbar_r_;
    neurodraw::Rectangle* cbar_c_;

    float f_treshold_l_ = 0.9f;
    float f_treshold_r_ = 0.9f;
    float f_treshold_c_ = 0.9f;  
};

} // namespace rosneuro

#endif
