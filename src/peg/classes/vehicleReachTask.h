#ifndef VEHICLEREACHTASK_H
#define VEHICLEREACHTASK_H

#include "task.h"
#include <ros/ros.h>
#include <cmat/cmat.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "../support/support.h"


class VehicleReachTask : public Task
{
public:
  VehicleReachTask(int dimension, int dof);
  VehicleReachTask(int dimension);
  ~VehicleReachTask();

private:
  int setActivation_impl();
  int setJacobian_impl(tf::StampedTransform wTv_tf);
  int setReference_impl(
      tf::StampedTransform wTv_tf, CMAT::TransfMatrix wTg_cmat);

};

#endif // VEHICLEREACHTASK_H
