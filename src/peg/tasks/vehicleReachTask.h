#ifndef VEHICLEREACHTASK_H
#define VEHICLEREACHTASK_H

#include "task.h" // task.h will include other header (like cmat and eigen)

class VehicleReachTask : public Task {

public:
  VehicleReachTask(int dimension, int dof);
  VehicleReachTask(int dimension);

  int updateMatrices(struct Transforms* const transf);

private:
  int setActivation();
  int setJacobian(Eigen::Matrix4d wTv_eigen);
  int setReference(
      Eigen::Matrix4d wTv_eigen, Eigen::Matrix4d wTg_eigen);

};

#endif // VEHICLEREACHTASK_H
