#include "../header/publisher.h"

Publisher::Publisher(ros::Publisher pub)
{
  this->pub = pub;

}

int Publisher::publish(geometry_msgs::TwistStamped twist){
  pub.publish(twist);

  return 0;
}


