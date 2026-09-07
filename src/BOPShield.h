#ifndef BOPSHIELD_H
#define BOPSHIELD_H

#include <Arduino.h>
#include <Servo.h>
#include "AutomationShield.h"
#include <lib/BasicLinearAlgebra/BasicLinearAlgebra.h>

#define _X1 A0
#define _X2 A2
#define _Y1 A1
#define _Y2 A3
#define _T1 13
#define _T2 12
#define _P A4
#define _S1 8
#define _S2 9

class BOPClass {
public:
  void begin();
  BLA::Matrix<2, 1> sensorRead();
  void calibration();
  int getvalueX();
  int getvalueY();
  int getvalueXmm();
  int getvalueYmm();
  void actuatorWrite(float motorX, float motorY);
  BLA::Matrix<2, 1> circle(float speed);
  BLA::Matrix<2, 1> oval(float speed);
  BLA::Matrix<2, 1> LQR(float x, float y, float Xsetpoint, float Ysetpoint, float Ts, float K11,float K12,float K13,
float K14, float K21, float K22, float K23, float K24);

  float PIDX(float x, float KpX, float KiX, float KdX,
             float Ts, float Xsetpoint);

  float PIDY(float y, float KpY, float KiY, float KdY,
             float Ts, float Ysetpoint);
  
private:
  int _X = 0;
  int _Y = 0;
  int _Xmm = 0;
  int _Ymm = 0;

  float errorX = 0.0;
  float errorY = 0.0;
  float error_prevX_PID = 0.0;
  float error_prevY_PID = 0.0;
  float x_prev_LQR=0.0;
  float y_prev_LQR=0.0;
  float error_sumX = 0.0;
  float error_sumY = 0.0;
  float errorX_speed = 0.0;
  float errorY_speed = 0.0;

  float uX = 0.0;
  float uY = 0.0;

  unsigned long lastTimeX = 0;
  unsigned long lastTimeY = 0;
  unsigned long lastTimeCircle = 0;
  unsigned long lastTimeOval = 0;
  unsigned long lastTimeLQR = 0;
  float timeInterval;

  int i = 0;

uint8_t CircleX[32] = {
  66, 66, 65, 63,
  62, 59, 57, 54,
  51, 48, 45, 43,
  40, 39, 37, 36,
  36, 36, 37, 39,
  40, 43, 45, 48,
  51, 54, 57, 59,
  62, 63, 65, 66
};

uint8_t CircleY[32] = {
  30, 33, 36, 38,
  41, 42, 44, 45,
  45, 45, 44, 42,
  41, 38, 36, 33,
  30, 27, 24, 22,
  19, 18, 16, 15,
  15, 15, 16, 18,
  19, 22, 24, 27
};

uint8_t OvalX[32] = {
  71, 71, 70, 68,
  66, 62, 59, 55,
  51, 47, 43, 40,
  36, 34, 32, 31,
  31, 31, 32, 34,
  36, 40, 43, 47,
  51, 55, 59, 62,
  66, 68, 70, 71
};

uint8_t OvalY[32] = {
  30, 32, 34, 36,
  37, 38, 39, 40,
  40, 40, 39, 38,
  37, 36, 34, 32,
  30, 28, 26, 24,
  23, 22, 21, 20,
  20, 20, 21, 22,
  23, 24, 26, 28
};
  float _Xcenter = 508;
  float _Ycenter = 482;

  float _Xmax = 948;
  float _Xmin = 89;
  float _Ymax = 842;
  float _Ymin = 137;

  BLA::Matrix<2, 1> _XY;
  BLA::Matrix<2, 1> _XYsetpointCircle;
  BLA::Matrix<2, 1> _XYsetpointOval;
  BLA::Matrix<2, 1> _XY_LQR;

  Servo _ServoOne;
  Servo _ServoTwo;
};

extern BOPClass BOPShield;

#endif