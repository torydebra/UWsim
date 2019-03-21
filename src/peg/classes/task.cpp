#include "task.h"

Task::Task(int dimension, int dof)
{

  J = CMAT::Matrix::Zeros(dimension, dof);
  A = CMAT::Matrix::Zeros(dimension, dimension);
  reference = CMAT::Matrix::Zeros(dimension,1);

}
