#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <cmat/cmat.h>
#include <tf/tf.h>

struct Transforms {

  // for vehicle
  CMAT::TransfMatrix wTgoal_cmat;
  tf::StampedTransform wTv_tf;

};

#endif
