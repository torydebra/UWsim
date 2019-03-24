#include "vehicleReachTask.h"

VehicleReachTask::VehicleReachTask(int dimension, int dof)
  : Task(dimension, dof) {}

VehicleReachTask::VehicleReachTask(int dimension)
  : Task(dimension) {}


int VehicleReachTask::updateMatrices(struct Transforms* const transf){

  setActivation();
  setJacobian(transf->wTv_eigen);
  setReference(transf->wTv_eigen, transf->wTgoal_eigen);
}

int VehicleReachTask::setJacobian(Eigen::Matrix4d wTv_eigen){

  Eigen::MatrixXd jacobian_eigen = Eigen::MatrixXd::Zero(dimension, dof);
  Eigen::Matrix3d wRv_eigen = wTv_eigen.topLeftCorner(3,3);

  //matrix([1:6];[1:4]) deve restare zero
  //matrix([1:3];[5:7]) parte linear
  jacobian_eigen.block(0,4, 3,3) = wRv_eigen; //block: beginning row, beginning column,  dimensions (rows & columns)

  //matrix([4:6];[8:10]) parte angolare
  //according to eigen doc, using these kind of specific function (and not
  //.block) improves performance
  jacobian_eigen.bottomRightCorner(3,3) = wRv_eigen;

  //to cmat
  //eigen unroll to vector for cmat function
  this->J = CMAT::Matrix(dimension, dof, jacobian_eigen.data());

}

int VehicleReachTask::setActivation(){

  double vectDiag[6];
  std::fill_n(vectDiag, 6, 1);
  this->A.SetDiag(vectDiag);

}

int VehicleReachTask::setReference(
    Eigen::Matrix4d wTv_eigen, Eigen::Matrix4d wTg_eigen){

  CMAT::TransfMatrix wTv_cmat = CONV::matrix_eigen2cmat(wTv_eigen);
  CMAT::TransfMatrix wTg_cmat = CONV::matrix_eigen2cmat(wTg_eigen);
  CMAT::Vect6 error = CMAT::CartError(wTg_cmat, wTv_cmat);
  double k = 0.2;
  this->reference = k * (error); //ang,lin
}
