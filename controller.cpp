#include "controller.h"
#include "tinympc/tiny_api.hpp"


Controller::Controller(BLA::Matrix<1, systemOrder> gains){
  this->controllerType = ControllerType::STATE_FEEDBACK;
  this->K = gains;
};

Controller::Controller(BLA::Matrix<systemOrder, systemOrder> A, BLA::Matrix<systemOrder,1> B, tinytype* Q_data, tinytype* R_data){
  this->controllerType = ControllerType::MPC;

  for(int i = 0; i < systemOrder; i++){
    for(int j = 0; j < systemOrder; j++){
      this->Adyn_data[i+j] = A(i, j);
    }
    this->Bdyn_data[i+0] = B(i, 0);
    this->Q_data[i] = Q_data[i];
  }
  
  this->R_data[0] = R_data[0];

  this->Adyn = Map<Matrix<tinytype, NSTATES, NSTATES, RowMajor>>(Adyn_data);
  this->Bdyn = Map<Matrix<tinytype, NSTATES, NINPUTS>>(Bdyn_data);
  this->Q = Map<Matrix<tinytype, NSTATES, 1>>(Q_data);
  this->R = Map<Matrix<tinytype, NINPUTS, 1>>(R_data);

  int status = tiny_setup(&this->solver,
                          Adyn, Bdyn, Q.asDiagonal(), R.asDiagonal(),
                          rho_value, NSTATES, NINPUTS, NHORIZON,
                          x_min, x_max, u_min, u_max, 1);

  // Update whichever settings we'd like
  this->solver->settings->max_iter = MAX_ITERATIONS;

  // Alias this->solver->work for brevity
  work = this->solver->work;

  // Reference
  work->Xref  << 0.0, 0.0; // TODO: check this latter
}

Controller::~Controller(){};

float Controller::controlLaw(BLA::Matrix<systemOrder,1> currentState){

  if(this->controllerType == ControllerType::MPC) {
    this->x << currentState(0), currentState(1); // current measurement

    // 1. Update measurement
    tiny_set_x0(this->solver, this->x);

    // 2. Solve MPC problem
    tiny_solve(this->solver);

    return this->work->u.col(0)[0];
  }
  
  return (this->K * currentState)(0);
}

void saturate(float* input, float lowerLimit, float upperLimit){
  if(*input < lowerLimit) *input = lowerLimit;
  if(*input > upperLimit) *input = upperLimit;
}
