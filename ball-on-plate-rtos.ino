#include "touchScreen.h"
#include "servoControl.h"
#include "controller.h"
#include "kalmanFilter.h"
#include "stateSpaceMatrices.h"
#include "tinympc/tiny_api.hpp"

#define USE_MPC

ServoControl servos(23, 19);
TouchScreen ts(27, 26, 32, 33, 25);
screenCoordinates coords;
screenCoordinatesCm coordsCm;

KalmanFilter xFilter(sys.A, sys.B, sys.C, 0.01, 200, 150);
KalmanFilter yFilter(sys.A, sys.B, sys.C, 0.01, 200, 150);

BLA::Matrix<1,2> hInfSatGains = {17.8954, 10.0515};

tinytype Q_data[NSTATES] = {10.0, 1.0};
tinytype R_data[NINPUTS] = {1.0};

#ifdef USE_MPC
Controller xController(sys.A, sys.B, Q_data, R_data);
Controller yController(sys.A, sys.B, Q_data, R_data);
#else
Controller xController(hInfSatGains);
Controller yController(hInfSatGains);
#endif

float angleX = 0;
float angleY = 0;

float posX = 0;
float posY = 0;

float uX = 0;
float uY = 0;

float uDegreeX = 0;
float uDegreeY = 0;

float referenceX = 0;
float referenceY = 0;

BLA::Matrix<2, 1> statesX = {0, 0};
BLA::Matrix<2, 1> statesY = {0, 0};

void setup() {
  delay(500);
  Serial.begin(1000000);
  delay(500);
  ts.setSamplingTime(35);
  servos.startPosition();
  delay(500);
}

//This is here purely for convenience. Otherwise the functions defined inside won't know about the variables.
#include "printInfo.h" 

void loop() {
  coords = ts.getCoordinates();
 
  if(ts.screenUpdated()){
    
    coordsCm = ts.getCoordinatesCm(coords.x, coords.y);
    
    posX = coordsCm.xCm * 0.01; // meter
    posY = -coordsCm.yCm * 0.01; // meter

    statesX = xFilter.kalman(uX, posX);
    statesY = yFilter.kalman(uY, posY);

    uX = xController.controlLaw(statesX);   
    uDegreeX = rad2deg(uX);
    saturate(&uDegreeX, -25, 25);
    uX = deg2rad(uDegreeX);

    uY = yController.controlLaw(statesY);   
    uDegreeY = rad2deg(uY);
    saturate(&uDegreeY, -25, 25);
    uY = deg2rad(uDegreeY);

    angleX = (uDegreeX) + servos.offset1;
    angleY = (uDegreeY) + servos.offset2;

    servos.moveServos(angleX , angleY);
    printXY();
  }
}
