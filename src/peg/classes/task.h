#ifndef TASK_H
#define TASK_H

#include <ros/ros.h>
#include <cmat/cmat.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>


class Task
{
public:

  virtual ~Task();

  CMAT::Matrix getJacobian();
  CMAT::Matrix getActivation();
  CMAT::Matrix getReference();

  //note why &:
  // value passed to cmar regPseudoinv that must be lvalue because:
  //Matrix& Matrix::RegPseudoInverse(double threshold, double lambda, double& mu, int& flag) const
  // these gets are used only to pass values to that cmat function
  int& getFlag_W();
  double& getMu_W();
  int& getFlag_G();
  double& getMu_G();

  int getThreshold();
  int getLambda();
  int getDof();
  int getDimension();

  //Public Overloaded Non-Virtuals Call Protected Non-Overloaded Virtuals
  int setJacobian(tf::StampedTransform wTv_tf);
  int setActivation();
  int setReference(tf::StampedTransform wTv_tf, CMAT::TransfMatrix wTg_cmat);


protected:
  int dimension; //dimension of the task (1 for scalar task); (rows)
  int dof; //total dofs of the robot (columns)
  CMAT::Matrix J; // the Jacobian
  CMAT::Matrix A; //the Activation function
  CMAT::Matrix reference;

  int flag_W;
  double mu_W;
  int flag_G;
  double mu_G;

  int threshold;
  int lambda;

  Task(int dimension, int dof);
  Task(int dimension);


  virtual int setJacobian_impl(tf::StampedTransform) = 0;
  virtual int setActivation_impl() = 0;
  virtual int setReference_impl(tf::StampedTransform, CMAT::TransfMatrix) = 0;


};

#endif // TASK_H
