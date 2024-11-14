void printX(){
  Serial.printf("realX:%f,filterX:%f,speedX:%f,uX:%f\n", posX * 100, statesX(0) * 100, statesX(1) * 100, uDegreeX);
};

void printY(){
  Serial.printf("realY:%f,filterY:%f,speedY:%f,uY:%f\n", posY * 100, statesY(0) * 100, statesY(1) * 100, uDegreeY);
};

void printXY(){
  Serial.printf("X:%f,Y:%f,Xreal:%f,Yreal:%f,bound:%i,boundN:%i\n", statesX(0) * 100, statesY(0) * 100, posX * 100, posY * 100, 1,-1);
}

void printXYu(){
  Serial.printf("X:%f,Y:%f,uX:%f,uY:%f\n", statesX(0) * 100, statesY(0) * 100, uDegreeX, uDegreeY);
}

void sendKalmanData(){
  Serial.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", 
    coords.x,
    posX * 100, 
    statesX(0) * 100, 
    xFilter.getPTrace(), 
    coords.y,
    posY * 100,
    statesY(0) * 100,  
    yFilter.getPTrace()
  );
}

void sendSystemData(){
  Serial.printf("%f,%f,%f,%f,%f,%f,%f,%f\n", 
  posX * 100,
  statesX(0) * 100, 
  statesX(1) * 100, 
  uDegreeX, 
  posY * 100,
  statesY(0) * 100, 
  statesY(1) * 100, 
  uDegreeY
  );  
}
