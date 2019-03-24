#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cmat/cmat.h>

#include "../support/defines.h"
#include "transforms.h"

#include "../tasks/task.h"
#include "../tasks/vehicleReachTask.h"


class Controller {

public:

  // the list of task. Each *tasks point to a different class (that is the specific task)
  //Task** tasks;
  std::vector<Task*> tasks;
  int numTasks;

  Controller();
  ~Controller();


  int updateTransforms(struct Transforms* const transf);
  CMAT::Matrix execAlgorithm();



private:
  int equalityIcat(Task* task, CMAT::Matrix* rhop, CMAT::Matrix* Q);
  int inequalityIcat(Task* task, CMAT::Matrix* rhop, CMAT::Matrix* Q) ;
};


#endif // CONTROLLER_H
