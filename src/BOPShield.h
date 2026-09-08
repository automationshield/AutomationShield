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
#define _P  A4
#define _S1 8
#define _S2 9

class BOPClass
{
public:

  void begin()
  {
    pinMode(_T1, INPUT);
    pinMode(_T2, INPUT);
    pinMode(_P, INPUT);

    _ServoOne.attach(_S1);
    _ServoTwo.attach(_S2);

    delay(500);
  }


  BLA::Matrix<2, 1> sensorRead()
  {
    _XY(0) = getvalueXmm();
    _XY(1) = getvalueYmm();

    return _XY;
  }


  int getvalueX()
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


  int getvalueY()
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


  int getvalueXmm()
  {
    int Xraw = getvalueX();

    _Xmm = map(Xraw, _Xmin, _Xmax, 0, 103);

    return _Xmm;
  }


  int getvalueYmm()
  {
    int Yraw = getvalueY();

    _Ymm = map(Yraw, _Ymin, _Ymax, 0, 60);

    return _Ymm;
  }


  void calibration()
  {
    _ServoOne.write(58);
    _ServoTwo.write(153);
  }


  void actuatorWrite(float motorX, float motorY)
  {
    _ServoOne.write(64 + motorY);
    _ServoTwo.write(153 - motorX);
  }


  BLA::Matrix<2, 1> circle(float speed)
  {
    timeInterval = map(speed, 0, 1023, 100, 150);

    if (millis() - lastTimeCircle > timeInterval)
    {
      _XYsetpointCircle(0) = CircleX[i];
      _XYsetpointCircle(1) = CircleY[i];

      i++;

      if (i == 32)
      {
        i = 0;
      }

      lastTimeCircle = millis();
    }

    return _XYsetpointCircle;
  }


  BLA::Matrix<2, 1> oval(float speed)
  {
    timeInterval = map(speed, 0, 1023, 100, 150);

    if (millis() - lastTimeOval > timeInterval)
    {
      _XYsetpointOval(0) = OvalX[i];
      _XYsetpointOval(1) = OvalY[i];

      i++;

      if (i == 32)
      {
        i = 0;
      }

      lastTimeOval = millis();
    }

    return _XYsetpointOval;
  }


  float PIDX(
    float x,
    float KpX,
    float KiX,
    float KdX,
    float Ts,
    float Xsetpoint
  )
  {
    if (millis() - lastTimeX >= Ts * 1000)
    {
      lastTimeX = millis();

      errorX = Xsetpoint - x;

      error_sumX = error_sumX + errorX * Ts;

      if (error_sumX > 100)
      {
        error_sumX = 100;
      }
      else if (error_sumX < -100)
      {
        error_sumX = -100;
      }

      uX =
        KpX * errorX +
        KiX * error_sumX +
        KdX * (errorX - error_prevX_PID) / Ts;

      if (uX > 10)
      {
        uX = 10;
      }
      else if (uX < -10)
      {
        uX = -10;
      }

      error_prevX_PID = errorX;
    }

    return uX;
  }


  float PIDY(
    float y,
    float KpY,
    float KiY,
    float KdY,
    float Ts,
    float Ysetpoint
  )
  {
    if (millis() - lastTimeY >= Ts * 1000)
    {
      lastTimeY = millis();

      errorY = Ysetpoint - y;

      error_sumY = error_sumY + errorY * Ts;

      if (error_sumY > 100)
      {
        error_sumY = 100;
      }
      else if (error_sumY < -100)
      {
        error_sumY = -100;
      }

      uY =
        KpY * errorY +
        KiY * error_sumY +
        KdY * (errorY - error_prevY_PID) / Ts;

      if (uY > 10)
      {
        uY = 10;
      }
      else if (uY < -10)
      {
        uY = -10;
      }

      error_prevY_PID = errorY;
    }

    return uY;
  }


  BLA::Matrix<2, 1> LQR(
    float x,
    float y,
    float Xsetpoint,
    float Ysetpoint,
    float Ts,
    float K11,
    float K12,
    float K13,
    float K14,
    float K21,
    float K22,
    float K23,
    float K24
  )
  {
    if (millis() - lastTimeLQR >= Ts * 1000)
    {
      lastTimeLQR = millis();

      float errorX = Xsetpoint - x;
      float errorY = Ysetpoint - y;

      float x_speed = (x - x_prev_LQR) / Ts;
      float y_speed = (y - y_prev_LQR) / Ts;

      _XY_LQR(0) =
        K11 * errorX -
        K12 * x_speed +
        K13 * errorY -
        K14 * y_speed;

      _XY_LQR(1) =
        K21 * errorX -
        K22 * x_speed +
        K23 * errorY -
        K24 * y_speed;

      if (_XY_LQR(0) > 10)
      {
        _XY_LQR(0) = 10;
      }
      else if (_XY_LQR(0) < -10)
      {
        _XY_LQR(0) = -10;
      }

      if (_XY_LQR(1) > 10)
      {
        _XY_LQR(1) = 10;
      }
      else if (_XY_LQR(1) < -10)
      {
        _XY_LQR(1) = -10;
      }

      x_prev_LQR = x;
      y_prev_LQR = y;
    }

    return _XY_LQR;
  }


private:

  int _X = 0;
  int _Y = 0;

  int _Xmm = 0;
  int _Ymm = 0;


  float errorX = 0.0;
  float errorY = 0.0;

  float error_prevX_PID = 0.0;
  float error_prevY_PID = 0.0;

  float x_prev_LQR = 0.0;
  float y_prev_LQR = 0.0;

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

  float timeInterval = 0.0;


  int i = 0;


  uint8_t CircleX[32] =
  {
    66, 66, 65, 63,
    62, 59, 57, 54,
    51, 48, 45, 43,
    40, 39, 37, 36,
    36, 36, 37, 39,
    40, 43, 45, 48,
    51, 54, 57, 59,
    62, 63, 65, 66
  };


  uint8_t CircleY[32] =
  {
    30, 33, 36, 38,
    41, 42, 44, 45,
    45, 45, 44, 42,
    41, 38, 36, 33,
    30, 27, 24, 22,
    19, 18, 16, 15,
    15, 15, 16, 18,
    19, 22, 24, 27
  };


  uint8_t OvalX[32] =
  {
    71, 71, 70, 68,
    66, 62, 59, 55,
    51, 47, 43, 40,
    36, 34, 32, 31,
    31, 31, 32, 34,
    36, 40, 43, 47,
    51, 55, 59, 62,
    66, 68, 70, 71
  };


  uint8_t OvalY[32] =
  {
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


/*
 * Header-only instance.
 *
 * "static" prevents linker multiple-definition errors if this header
 * is included from more than one compilation unit.
 */
static BOPClass BOPShield;


#endif