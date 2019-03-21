#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "../header/publisher.h"
#include <cmat/cmat.h>
#include "../classes/task.h"
#include "../support/support.h"
#include "../support/defines.h"

int main(int argc, char **argv)
{
  ROS_INFO("[CONTROLLER] Start");
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;

  std::string topicTwist = "/uwsim/g500_A/twist_command_A";
  ros::Publisher pubTwist = nh.advertise<geometry_msgs::TwistStamped>(topicTwist,1);


  Publisher pubClassTwist(pubTwist);
  geometry_msgs::TwistStamped twist;
  twist.twist.linear.x=0;
  twist.twist.linear.y=0;
  twist.twist.linear.z=0;
  twist.twist.angular.x=0;
  twist.twist.angular.y=0;
  twist.twist.angular.z=0;

  // coord vehicle near the peg [-0.287, -0.062, 7.424]
  //tf::Vector3 goalPoint = tf::Vector3(-5.287, -0.062, 7.424); //respect world
  //tf::Transform goal = tf::Transform(tf::Matrix3x3(1, 0, 0,  0, 1, 0,  0, 0, 1), goalPoint);
  tf::Vector3 qDot = tf::Vector3(0, 0, 0);
  //tf::Vector3 xDot = tf::Vector3(0, 0, 0);

  /// GOAL

  double goalLine[3] = {-5.287, -0.062, 7.424};
  CMAT::Vect3 goal_xyz(goalLine);
  CMAT::RotMatrix goal_rot = CMAT::Matrix::Eye(3);
  CMAT::TransfMatrix wTgoal (goal_rot, goal_xyz);


  //TRANSFORM LISTENER ROBE
  tf::TransformListener tfListener;
  tf::StampedTransform wTv_tf;

  //Wait to transform to be ready (or fail if wait more than 3 sec)
  tfListener.waitForTransform("world", "/girona500_A", ros::Time(0), ros::Duration(3.0));


  //initialize task
  Task vehicleReaching(6,10); //4DOF ARM

  ros::Rate r(1000); //1Hz
  while(ros::ok()){
    try {

    tfListener.lookupTransform("world", "/girona500_A", ros::Time(0), wTv_tf);

    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    /// Activation
    double vectDiag[6];
    std::fill_n(vectDiag, 6, 1);
    vehicleReaching.A.SetDiag(vectDiag);


    /** Jacobian */
   // double* vector = SUPPORT::tfMat_to_double(wTv.getBasis());

    Eigen::MatrixXd jacobian_eigen(6,10);
    jacobian_eigen = Eigen::MatrixXd::Zero(6,10);

    Eigen::Matrix3d wRv_Eigen;
    tf::matrixTFToEigen(wTv_tf.getBasis(), wRv_Eigen);

    //matrix([1:6];[1:4]) deve restare zero
    //matrix([1:3];[5:7]) parte linear
    jacobian_eigen.block(0,4, 3,3) = wRv_Eigen;

    //matrix([4:6];[8:10]) parte angolare
    //according to eigen doc, using these kind of specific function (and not
    //(.block improves performance
    jacobian_eigen.bottomRightCorner(3,3) = wRv_Eigen;

    // in pasto a cmat
    //eigen unroll to vector for cmat function
    vehicleReaching.J = CMAT::Matrix(6,TOT_DOF, jacobian_eigen.data());


    /// Reference
    //std::cout << wRv_Eigen << std::endl;
    CMAT::TransfMatrix wTv_cmat = CONV::transfMatrix_tf2cmat(wTv_tf);
    CMAT::Vect6 error = CMAT::CartError(wTgoal, wTv_cmat);
    double k = 0.2;
    vehicleReaching.reference = k * (error); //ang,lin


    /// PSEUDOINVERSE!!!!
    CMAT::Matrix qDot(TOT_DOF,1);
    //qdot = [arm arm arm arm wx wy wz x y z]
    qDot = vehicleReaching.J.RegPseudoInverse(0.001, 0.0001) *
                       vehicleReaching.reference;

   // vehicleReaching.J.RegPseudoInverse(0.001, 0.0001).PrintMtx();
    //vehicleReaching.reference.PrintMtx();


    twist.twist.angular.x=qDot(5);
    twist.twist.angular.y=qDot(6);
    twist.twist.angular.z=qDot(7);
    twist.twist.linear.x=qDot(8);
    twist.twist.linear.y=qDot(9);
    twist.twist.linear.z=qDot(10);

    pubClassTwist.publish(twist);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
