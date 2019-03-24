#ifndef ROSINTERFACE_H
#define ROSINTERFACE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>
#include "../support/support.h"
#include "../support/defines.h"


class RosInterface
{
public:
  RosInterface(std::string robotName, std::string topicTwist, int argc, char **argv);
  int init();
  int getwTv(Eigen::Matrix4d* wTv_eigen);
  int sendQDot(std::vector<double> qDot);


private:
  std::string robotName;
  ros::Publisher pubTwist;
  std::string topicTwist;
  tf::TransformListener tfListener_wTv;
  tf::StampedTransform wTv_tf;
};

#endif // ROSINTERFACE_H
