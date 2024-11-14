#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <BasicLinearAlgebra.h>
using namespace BLA;

class KalmanFilter{

  private:

  static const int systemOrder = 2;

  BLA::Matrix<systemOrder, systemOrder> A;
  BLA::Matrix<systemOrder,1> B;
  BLA::Matrix<1, systemOrder> C;
  
  BLA::Matrix<systemOrder, systemOrder> P_priori;
  BLA::Matrix<systemOrder, systemOrder> P_hat;
  
  BLA::Matrix<systemOrder, 1> x_hat;
  BLA::Matrix<systemOrder, 1> x_priori;

  BLA::Matrix<systemOrder, 1> K_kalman;
  
  //Identity BLA::Matrix
  const BLA::Matrix<systemOrder, systemOrder> U = {
    1, 0,
    0, 1
  };

  BLA::Matrix<systemOrder, systemOrder> Q;
  BLA::Matrix<1,1> R;
  
  public:

  KalmanFilter(BLA::Matrix<systemOrder, systemOrder> A, BLA::Matrix<systemOrder,1> B, BLA::Matrix<1, systemOrder> C, float qValue1, float qValue2, float rValue);
  ~KalmanFilter();
  float getPTrace();

  BLA::Matrix<systemOrder, 1> kalman(float input, float output);  
};

#endif
