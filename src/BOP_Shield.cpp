/* #include "BOP_Shield.h"

BOPClass BOPShield;

void BOPClass::begin()
{
  pinMode(_T1, INPUT);
  pinMode(_T2, INPUT);
  pinMode(_P, INPUT);

  _ServoOne.attach(_S1);
  _ServoTwo.attach(_S2);
  delay(500);
}
BLA::Matrix<2, 1> BOPClass::sensorRead()
{
  _XY(0) = getvalueXmm();
  _XY(1) = getvalueYmm();
  return _XY;
}

int BOPClass::getvalueX()
{
  pinMode(_X1, OUTPUT);
  pinMode(_X2, OUTPUT);

  pinMode(_Y1, INPUT);
  pinMode(_Y2, INPUT);

  digitalWrite(_X1, HIGH);
  digitalWrite(_X2, LOW);
  _X = analogRead(_Y1);
  return _X;
}

int BOPClass::getvalueY()
{
  pinMode(_Y1, OUTPUT);
  pinMode(_Y2, OUTPUT);

  pinMode(_X1, INPUT);
  pinMode(_X2, INPUT);

  digitalWrite(_Y1, HIGH);
  digitalWrite(_Y2, LOW);

  _Y = analogRead(_X1);

  return _Y;
}

int BOPClass::getvalueXmm()
{
  int Xraw = getvalueX();
  _Xmm = map(Xraw, _Xmin, _Xmax, 0, 103);
  return _Xmm;
}

int BOPClass::getvalueYmm()
{
  int Yraw = getvalueY();
  _Ymm = map(Yraw, _Ymin, _Ymax, 0, 60);
  return _Ymm;
}

void BOPClass::calibration()
{
  _ServoOne.write(58);
  _ServoTwo.write(153);
}

void BOPClass::actuatorWrite(float motorX, float motorY)
{
  _ServoOne.write(64+motorY);
  _ServoTwo.write(153-motorX);
}

BLA::Matrix<2, 1> BOPClass::circle(float speed)
{
 timeInterval= map(speed,0,1023,100,150);
  if(millis()-lastTimeCircle>timeInterval){
    _XYsetpointCircle(0)=CircleX[i];
    _XYsetpointCircle(1) =CircleY[i];
    i=i+1;
    if(i==32){
      i=0;
    }
    lastTimeCircle=millis();
  }
  return _XYsetpointCircle;
}
BLA::Matrix<2, 1> BOPClass::oval(float speed){
   timeInterval= map(speed,0,1023,100,150);  
   if(millis()-lastTimeOval>timeInterval){
    _XYsetpointOval(0)=OvalX[i];
    _XYsetpointOval(1) =OvalY[i];
    i=i+1;
    if(i==32){
      i=0;
    }
    lastTimeOval=millis();
  }
  return _XYsetpointOval;


}
float BOPClass::PIDX(float x, float KpX, float KiX, float KdX, float Ts, float Xsetpoint)
{
  if (millis() - lastTimeX >= Ts * 1000 ) {
    lastTimeX = millis();

    errorX = Xsetpoint - x;
    error_sumX = error_sumX + errorX * Ts;

    if (error_sumX > 100) error_sumX = 100;
    else if (error_sumX < -100) error_sumX = -100;
    
      uX = KpX * errorX + KiX * error_sumX + KdX * (errorX - error_prevX_PID) / Ts;
    
    if (uX > 10) {
      uX = 10;
    }
    else if (uX < -10) {
      uX = -10;
    }

    error_prevX_PID = errorX;
  }

  return uX;
}

float BOPClass::PIDY(float y, float KpY, float KiY, float KdY, float Ts, float Ysetpoint)
{

  if (millis() - lastTimeY >= Ts*1000) {
    lastTimeY = millis();

    errorY = Ysetpoint - y;
    error_sumY = error_sumY + errorY * Ts;

    if (error_sumY > 100) error_sumY = 100;
    else if (error_sumY < -100) error_sumY = -100;
  
      uY = KpY * errorY + KiY * error_sumY + KdY * (errorY - error_prevY_PID) / Ts;
      
    if (uY > 10) {
      uY = 10;
    }
    else if (uY < -10) {
      uY = -10;
    }

    error_prevY_PID = errorY;
  }

  return uY;
}
BLA::Matrix<2, 1> BOPClass::LQR(
  float x, float y,
  float Xsetpoint, float Ysetpoint,
  float Ts,
  float K11, float K12, float K13, float K14,
  float K21, float K22, float K23, float K24
) {
  if (millis() - lastTimeLQR >= Ts * 1000) {
    lastTimeLQR = millis();

    float errorX = Xsetpoint - x;
    float errorY = Ysetpoint - y;

    float x_speed = (x - x_prev_LQR) / Ts;
    float y_speed = (y - y_prev_LQR) / Ts;

    _XY_LQR(0) = K11 * errorX - K12 * x_speed + K13 * errorY - K14 * y_speed;
    _XY_LQR(1) = K21 * errorX - K22 * x_speed + K23 * errorY - K24 * y_speed;

    if (_XY_LQR(0) > 10) _XY_LQR(0) = 10;
    else if (_XY_LQR(0) < -10) _XY_LQR(0) = -10;

    if (_XY_LQR(1) > 10) _XY_LQR(1) = 10;
    else if (_XY_LQR(1) < -10) _XY_LQR(1) = -10;

    x_prev_LQR = x;
    y_prev_LQR = y;
  }

  return _XY_LQR;
}
  */