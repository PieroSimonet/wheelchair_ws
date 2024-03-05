#ifndef CV_EMAPC_H
#define CV_EMAPC_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace ema_pc {
  class EmaPc {
  public:
    EmaPc(ros::NodeHandle& nh);
    void run();
    void on_recived_poincloud(const PointCloud::ConstPtr& msg);
    void process_data();

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    PointCloud mean_cloud_;
    PointCloud new_cloud_;

    bool  new_data_;
    bool  first_run_;
    
    float ema_coeff_;
    int   hz_;
  };
}

#endif
