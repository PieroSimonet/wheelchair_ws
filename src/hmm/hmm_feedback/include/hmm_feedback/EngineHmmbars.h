#ifndef ROSNEURO_FEEDBACK_ENGINE_HMM_BARS_H_
#define ROSNEURO_FEEDBACK_ENGINE_HMM_BARS_H_

#include "hmm_feedback/bars.h"
#include "rosneuro_msgs/NeuroEvent.h"
#include "std_msgs/String.h"

namespace rosneuro {

const int INIT   =   1;

const int LEFT   = 101;
const int CENTER = 102;
const int RIGTH  = 103;

const int CORRECT_MASK = 1000;
const int WRONG_MASK   = 2000;
const int EXPIRED_MASK = 3000;

const int FIXATION   = 786;
const int CLOSE_MASK = 32768;

class bars_engine : public bars {
  public:
    enum class MODE {
      NO_WALL = 0,
      ONE_WALL,
      TWO_WALL,
      NONE
    };

    bars_engine(void);
    ~bars_engine(void);
  
  protected:
    void event_timecallback_cue(const ros::TimerEvent& event);

  private:
    void set_gazebo_env();

    int n_task;
    int c_task;
    int n_passed;
    int n_wrong;
    int n_time_expired;

    ros::Publisher event_pub;
    ros::Publisher string_pub;

    ros::Timer      timer_cue_;


};

} // namespace rosneuro

#endif
