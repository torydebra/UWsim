#ifndef SUPPORT_H
#define SUPPORT_H

#include <Eigen/Geometry>
#include <cmat/cmat.h>
#include <tf/tf.h>

namespace CONV{

    std::vector<double> tfMat3x3_to_vector(tf::Matrix3x3 matrix3x3);

    CMAT::RotMatrix rotMatrix_eigen2cmat(Eigen::Matrix3d mat_eigen);
    Eigen::Matrix3d rotMatrix_cmat2eigen(CMAT::RotMatrix mat_cmat);

    CMAT::TransfMatrix transfMatrix_eigen2cmat(Eigen::Matrix4d mat_eigen);
    Eigen::Matrix4d transfMatrix_cmat2eigen(CMAT::TransfMatrix mat_cmat);


    CMAT::TransfMatrix transfMatrix_tf2cmat(tf::StampedTransform mat_tf);

}

namespace PRINT{
  int printRotMatrix_tf(tf::StampedTransform transform);
  int printMatrix3x3_tf(tf::Matrix3x3 matrix);
}

#endif // SUPPORT_H
