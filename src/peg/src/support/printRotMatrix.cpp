#include <ros/ros.h>
#include <tf/transform_listener.h>

int printRotMatrix(tf::StampedTransform transform){

  std::cout << transform.getBasis().getColumn(0).getX() << "\t"
            << transform.getBasis().getColumn(1).getX() << "\t"
            << transform.getBasis().getColumn(2).getX() << "\n"
            << transform.getBasis().getColumn(0).getY() << "\t"
            << transform.getBasis().getColumn(1).getY() << "\t"
            << transform.getBasis().getColumn(2).getY() << "\n"
            << transform.getBasis().getColumn(0).getZ() << "\t"
            << transform.getBasis().getColumn(1).getZ() << "\t"
            << transform.getBasis().getColumn(2).getZ() << "\n\n";
  return 0;

}
