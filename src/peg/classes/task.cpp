#include "task.h"
#include "../support/defines.h"

Task::Task(int dimension, int dof)
{

  this->dimension = dimension;
  this->dof = dof;

  J = CMAT::Matrix::Zeros(dimension, dof);
  A = CMAT::Matrix::Zeros(dimension, dimension);
  reference = CMAT::Matrix::Zeros(dimension,1);

  threshold = THRESHOLD_DEFAULT;
  lambda = LAMBDA_DEFAULT;

  //for reg pseudoinverse di cmat
  flag_W = 0;
  mu_W = 0.0;
  flag_G = 0;
  mu_G = 0.0;

}

Task::Task(int dimension)
{

  this->dimension = dimension;
  this->dof = TOT_DOF;

  J = CMAT::Matrix::Zeros(dimension, dof);
  A = CMAT::Matrix::Zeros(dimension, dimension);
  reference = CMAT::Matrix::Zeros(dimension,1);

  threshold = THRESHOLD_DEFAULT;
  lambda = LAMBDA_DEFAULT;

  //for reg pseudoinverse di cmat
  flag_W = 0;
  mu_W = 0.0;
  flag_G = 0;
  mu_G = 0.0;

}

Task::~Task(){}


CMAT::Matrix Task::getJacobian(){ return this->J;}
CMAT::Matrix Task::getActivation(){return this->A;}
CMAT::Matrix Task::getReference(){return this->reference;}

//note why &:
// value passed to cmar regPseudoinv that must be lvalue because:
//Matrix& Matrix::RegPseudoInverse(double threshold, double lambda, double& mu, int& flag) const
// these gets are used only to pass values to that cmat function
int& Task::getFlag_W(){return this->flag_W;}
double& Task::getMu_W(){return this->mu_W;}
int& Task::getFlag_G(){return this->flag_G;}
double& Task::getMu_G(){return this->mu_G;}

int Task::getThreshold(){return this->threshold;}
int Task::getLambda(){return this->lambda;}
int Task::getDof(){return this->dof;}
int Task::getDimension(){return this->dimension;}

//Public Overloaded Non-Virtuals Call Protected Non-Overloaded Virtuals
int Task::setJacobian(tf::StampedTransform wTv_tf){
  setJacobian_impl(wTv_tf);
}

int Task::setActivation(){
  setActivation_impl();
}

int Task::setReference(
    tf::StampedTransform wTv_tf, CMAT::TransfMatrix wTg_cmat){

  setReference_impl(wTv_tf, wTg_cmat);
}
