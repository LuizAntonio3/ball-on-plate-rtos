#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <BasicLinearAlgebra.h>
#include "tiny_api.hpp"

#define NSTATES 2
#define NINPUTS 1
#define NHORIZON 30
#define MAX_ITERATIONS 200

typedef Matrix<tinytype, NINPUTS, NHORIZON-1> tiny_MatrixNuNhm1;
typedef Matrix<tinytype, NSTATES, NHORIZON> tiny_MatrixNxNh;
typedef Matrix<tinytype, NSTATES, 1> tiny_VectorNx;

void saturate(float* input, float lowerLimit, float upperLimit);

enum ControllerType {
  STATE_FEEDBACK,
  MPC
};

class Controller{
private:
  static const int systemOrder = 2;
  BLA::Matrix<1, systemOrder> K; 

  ControllerType controllerType;

  // MPC related
  TinySolver *solver;
  TinyWorkspace *work;

  float rho_value = 1.0; // verificar o que é isso

  tinytype Adyn_data[NSTATES * NSTATES];
  tinytype Bdyn_data[NSTATES * NINPUTS];
  tinytype Q_data[NSTATES];
  tinytype R_data[NINPUTS];

  tinyMatrix Adyn;
  tinyMatrix Bdyn;
  tinyVector Q;
  tinyVector R;

  tinyMatrix x_min = tiny_MatrixNxNh::Constant(-100);  // fix this values
  tinyMatrix x_max = tiny_MatrixNxNh::Constant(100);   // fix this values
  tinyMatrix u_min = tiny_MatrixNuNhm1::Constant(-20); // fix this values
  tinyMatrix u_max = tiny_MatrixNuNhm1::Constant(20);  // fix this values

  tiny_VectorNx x; // stores the states -> gets from the kalman filter
public:
  Controller(BLA::Matrix<systemOrder, systemOrder> A, BLA::Matrix<systemOrder,1> B, tinytype Q_data[NSTATES], tinytype R_data[NSTATES]); // Constructor for the MPC
  Controller(BLA::Matrix<1, systemOrder> gains);
  ~Controller();
  float controlLaw(BLA::Matrix<systemOrder, 1> currentState);
};
#endif
