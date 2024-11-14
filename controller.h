#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <BasicLinearAlgebra.h>
#include "tinympc/tiny_api.hpp"

#define NSTATES 4
#define NINPUTS 1

#define NHORIZON 10

void saturate(float* input, float lowerLimit, float upperLimit);

class Controller{

  private:
  static const int systemOrder = 2;
  BLA::Matrix<1, systemOrder> K; 
  public:  
  Controller(BLA::Matrix<1, systemOrder> gains);
  ~Controller();
  float controlLaw(BLA::Matrix<systemOrder, 1> currentState);
};
#endif
