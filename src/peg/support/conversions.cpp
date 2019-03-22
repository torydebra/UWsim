#include "support.h"

std::vector<double> CONV::tfMat3x3_to_vector(tf::Matrix3x3 matrix3x3){

  std::vector<double> vector(9);
  vector[0] = matrix3x3.getColumn(0).getX();
  vector[1] = matrix3x3.getColumn(0).getY();
  vector[2] = matrix3x3.getColumn(0).getZ();
  vector[3] = matrix3x3.getColumn(1).getX();
  vector[4] = matrix3x3.getColumn(1).getY();
  vector[5] = matrix3x3.getColumn(1).getZ();
  vector[6] = matrix3x3.getColumn(2).getX();
  vector[7] = matrix3x3.getColumn(2).getY();
  vector[8] = matrix3x3.getColumn(2).getZ();

  return vector;

}


Eigen::Matrix3d CONV::rotMatrix_cmat2eigen(CMAT::RotMatrix mat_cmat){

  Eigen::Matrix3d mat_eigen;
  mat_eigen << mat_cmat(1,1), mat_cmat(1,2), mat_cmat(1,3),
               mat_cmat(2,1), mat_cmat(2,2), mat_cmat(2,3),
               mat_cmat(3,1), mat_cmat(3,2), mat_cmat(3,3);

  return mat_eigen;
}


Eigen::Matrix4d CONV::transfMatrix_cmat2eigen(CMAT::TransfMatrix mat_cmat){

  Eigen::Matrix4d mat_eigen;
  mat_eigen << mat_cmat(1,1), mat_cmat(1,2), mat_cmat(1,3), mat_cmat(1,4),
               mat_cmat(2,1), mat_cmat(2,2), mat_cmat(2,3), mat_cmat(2,4),
               mat_cmat(3,1), mat_cmat(3,2), mat_cmat(3,3), mat_cmat(3,4),
               mat_cmat(4,1), mat_cmat(4,2), mat_cmat(4,3), mat_cmat(4,4);

  return mat_eigen;
}

CMAT::TransfMatrix CONV::transfMatrix_tf2cmat(tf::StampedTransform mat_tf){

  std::vector<double> vector = CONV::tfMat3x3_to_vector(mat_tf.getBasis());
  CMAT::RotMatrix rot_cmat(&vector[0]);
  double* lin = mat_tf.getOrigin();
  CMAT::Vect3 orient_cmat(lin);

  CMAT::TransfMatrix mat_cmat(rot_cmat, orient_cmat);

  return mat_cmat;
}

CMAT::Matrix CONV::matrix_eigen2cmat(Eigen::MatrixXd mat_eigen){
  CMAT::Matrix mat_cmat(mat_eigen.rows(), mat_eigen.cols(), mat_eigen.data());
  return mat_cmat;
}

