#include "support.h"

std::vector<double> CONV::tfMat3x3_to_vector(tf::Matrix3x3 matrix3x3){

  std::vector<double> array(9);
  array[0] = matrix3x3.getColumn(0).getX();
  array[1] = matrix3x3.getColumn(0).getY();
  array[2] = matrix3x3.getColumn(0).getZ();
  array[3] = matrix3x3.getColumn(1).getX();
  array[4] = matrix3x3.getColumn(1).getY();
  array[5] = matrix3x3.getColumn(1).getZ();
  array[6] = matrix3x3.getColumn(2).getX();
  array[7] = matrix3x3.getColumn(2).getY();
  array[8] = matrix3x3.getColumn(2).getZ();

  return array;

}

CMAT::RotMatrix CONV::rotMatrix_eigen2cmat(Eigen::Matrix3d mat_eigen){

  CMAT::RotMatrix mat_cmat(mat_eigen.data());
  return mat_cmat;
}

Eigen::Matrix3d CONV::rotMatrix_cmat2eigen(CMAT::RotMatrix mat_cmat){

  Eigen::Matrix3d mat_eigen;
  mat_eigen << mat_cmat(1,1), mat_cmat(1,2), mat_cmat(1,3),
               mat_cmat(2,1), mat_cmat(2,2), mat_cmat(2,3),
               mat_cmat(3,1), mat_cmat(3,2), mat_cmat(3,3);

  return mat_eigen;
}

CMAT::TransfMatrix CONV::transfMatrix_eigen2cmat(Eigen::Matrix4d mat_eigen){

  CMAT::TransfMatrix mat_cmat(mat_eigen.data());
  return mat_cmat;
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

