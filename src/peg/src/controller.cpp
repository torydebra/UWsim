#include "../header/controller.h"


Controller::Controller() {

  //tasks = new Task*[NUM_TASKS]; //not necessary with tasks std::vector

  //note: std::vector is nicer because to change the order of priority or to leave for the moment
  // a task we can simply comment the row.
  //instead, with tasks as Task**, we need to fill the list with task[0], ...task[i] ... and changing
  //priority order would be slower.

  /// PUT HERE NEW TASKS. FOR THIS CLASS, ONLY MODIFICATIONS HERE ARE NECESSARY
  // note: order of priority at the moment is here
  tasks.push_back(new VehicleReachTask(6, TOT_DOF));



  // store number of task inserted
  numTasks = tasks.size();
}


///   TO NOT MODIFY BELOW TO ADD NEW TASK

//Important to delete all object pointed
Controller::~Controller(){
  for (std::vector< Task* >::iterator it = tasks.begin() ; it != tasks.end(); ++it)
    {
      delete (*it);
    }
    tasks.clear();
}

int Controller::updateTransforms(struct Transforms* const transf){

  for (int i=0; i<numTasks; i++){
    tasks[i]->updateMatrices(transf);
  }
}

std::vector<double> Controller::execAlgorithm(){

  //initialize qdot and Q for algorithm
  //qdot = [arm arm arm arm wx wy wz x y z]
  CMAT::Matrix qDot_cmat = CMAT::Matrix (TOT_DOF,1);
  CMAT::Matrix Q = CMAT::Matrix::Eye(TOT_DOF);

  for (int i=0; i<numTasks; i++){
    Controller::equalityIcat(tasks[i], &qDot_cmat, &Q);
  }

  std::vector<double> qDot_vect(TOT_DOF);
  int i = 1;
  for(std::vector<double>::iterator it = qDot_vect.begin(); it != qDot_vect.end(); ++it) {
    *it = qDot_cmat(i);
    i++;
  }

  return qDot_vect;

}

//TODO passare il task o le cose singole?
//int Equalitytask(CMAT::Matrix J, CMAT::Matrix Q, CMAT::Matrix rhop, CMAT::Matrix xdot,) {
int  Controller::equalityIcat(Task* task, CMAT::Matrix* rhop, CMAT::Matrix* Q) {

  CMAT::Matrix J = task->getJacobian(); //J jacobiana task
  CMAT::Matrix ref = task->getReference(); //  reference del task

  //Q inizializzata a eye(totDof) prima di fare tutto
  //rhop il controllo cinematico che viene passato a tutti i lower priority task,
  //       che poi alla fine è qdot voluto

  //mu double var globale che viene modificata da cmat pseudoinverse varie
  //flag int analogo a mu
  //NOTA: sono doppie: una per calcolare la W una per la barGpinv

  CMAT::Matrix I = CMAT::Matrix::Eye(task->getDof());
  CMAT::Matrix barG, barGtransp, T, W, barGpinv;
  // barG the actual Jacobian
  barG = J * (*Q);
  barGtransp = barG.Transpose();

  /// Regularization matrices
  // T is the reg. matrix for the different levels of task priority
  // takes into account that not all the controls are available
  T = (I - (*Q)).Transpose() * (I-(*Q));

  // compute W to solve the problem of discontinuity due to different priority levels
  W = barG *
      (barGtransp * barG + T)
        .RegPseudoInverse(task->getThreshold(), task->getLambda(),
                          task->getMu_W(), task->getFlag_W()) *
      barGtransp;

  /// Compute the rho for this task
  barGpinv = (barGtransp * barG)
      .RegPseudoInverse(task->getThreshold(), task->getLambda(),
                        task->getMu_G(), task->getFlag_G());
  (*rhop) = (*rhop) + (*Q) * barGpinv * barGtransp * W * (ref - J * (*rhop));

  ///TODO: in ctrl_task_algo calcola anche un tmpProjector e una P_ che non so a che servono


  /// Update the projector matrix
  (*Q) = (*Q) * (I - barGpinv * barGtransp * barG);

  return 0;

}

int Controller::inequalityIcat(Task* task, CMAT::Matrix* rhop, CMAT::Matrix* Q) {

  CMAT::Matrix J = task->getJacobian(); //Jacobiana of task
  CMAT::Matrix ref = task->getReference(); // Reference of task
  CMAT::Matrix A = task->getActivation(); // Activation of task

  CMAT::Matrix I = CMAT::Matrix::Eye(task->getDof());
  CMAT::Matrix barG, barGtransp, barGtranspAA, T, H, W, barGpinv;
  // barG the actual Jacobian
  barG = J * (*Q);
  barGtransp = barG.Transpose();
  barGtranspAA = barGtransp * A * A;

  /// Regularization matrices
  // T is the reg. matrix for the different levels of task priority
  // takes into account that not all the controls are available
  T = (I - (*Q)).Transpose() * (I-(*Q));
  // H is the reg. matrix for the activation, i.e. A*(I-A)
  H = barGtransp *
      (CMAT::Matrix::Eye(task->getDimension()) - A)*
      A * barG;

  // compute W to solve the problem of discontinuity due to different priority levels
  W = barG *
      (barGtranspAA * barG + T + H)
        .RegPseudoInverse(task->getThreshold(), task->getLambda(),
                          task->getMu_W(), task->getFlag_W()) *
      barGtranspAA;

  barGpinv = (barGtranspAA * barG + H)
               .RegPseudoInverse(task->getThreshold(), task->getLambda(),
                                 task->getMu_G(), task->getFlag_G());


  ///TODO: in ctrl_task_algo calcola anche un tmpProjector e una P_ che non so a che servono


  /// Compute the rho for this task
  (*rhop) = (*rhop) + (*Q) * barGpinv * barGtranspAA * W * (ref - J * (*rhop));

  /// Update the projector matrix
  (*Q) = (*Q) * (I - barGpinv * barGtranspAA * barG);

  return 0;
}

