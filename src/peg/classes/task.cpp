#include "task.h"
#include "../support/defines.h"

Task::Task(int dimension, int dof)
{

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
