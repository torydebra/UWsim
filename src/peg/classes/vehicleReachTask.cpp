#include "vehicleReachTask.h"

VehicleReachTask::VehicleReachTask(int dimension, int dof)
  : Task(dimension, dof) {}

VehicleReachTask::VehicleReachTask(int dimension)
  : Task(dimension) {}

VehicleReachTask::~VehicleReachTask(){}

int VehicleReachTask::setJacobian_impl(tf::StampedTransform wTv_tf){

  Eigen::MatrixXd jacobian_eigen(dimension, dof);
  jacobian_eigen = Eigen::MatrixXd::Zero(dimension, dof);

  Eigen::Matrix3d wRv_Eigen;
  tf::matrixTFToEigen(wTv_tf.getBasis(), wRv_Eigen);

  //matrix([1:6];[1:4]) deve restare zero
  //matrix([1:3];[5:7]) parte linear
  jacobian_eigen.block(0,4, 3,3) = wRv_Eigen;

  //matrix([4:6];[8:10]) parte angolare
  //according to eigen doc, using these kind of specific function (and not
  //.block) improves performance
  jacobian_eigen.bottomRightCorner(3,3) = wRv_Eigen;

  //to cmat
  //eigen unroll to vector for cmat function
  this->J = CMAT::Matrix(dimension, dof, jacobian_eigen.data());

}

int VehicleReachTask::setActivation_impl(){

  double vectDiag[6];
  std::fill_n(vectDiag, 6, 1);
  this->A.SetDiag(vectDiag);

}

int VehicleReachTask::setReference_impl(
    tf::StampedTransform wTv_tf, CMAT::TransfMatrix wTg_cmat){

  CMAT::TransfMatrix wTv_cmat = CONV::transfMatrix_tf2cmat(wTv_tf);
  CMAT::Vect6 error = CMAT::CartError(wTg_cmat, wTv_cmat);
  double k = 0.2;
  this->reference = k * (error); //ang,lin
}
