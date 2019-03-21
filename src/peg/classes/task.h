#ifndef TASK_H
#define TASK_H

#include <cmat/cmat.h>

class Task
{
public:
  CMAT::Matrix J; // the jacobian
  CMAT::Matrix A; //the Activation function
  CMAT::Vect6 reference;

  Task(int dimension, int dof);

};

#endif // TASK_H
