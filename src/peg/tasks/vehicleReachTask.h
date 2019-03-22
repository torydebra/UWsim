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
#include "../header/transforms.h"


class VehicleReachTask : public Task {

public:
  VehicleReachTask(int dimension, int dof);
  VehicleReachTask(int dimension);

  int updateMatrices(struct Transforms* const transf);

private:
  int setActivation();
  int setJacobian(tf::StampedTransform wTv_tf);
  int setReference(
      tf::StampedTransform wTv_tf, CMAT::TransfMatrix wTg_cmat);

};

#endif // VEHICLEREACHTASK_H
