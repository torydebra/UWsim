#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>


class Publisher
{
private:
  ros::Publisher pub;

public:
  Publisher(ros::Publisher pub);
  int publish(geometry_msgs::TwistStamped twist);
};

#endif // PUBLISHER_H
