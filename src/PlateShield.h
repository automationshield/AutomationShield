#ifndef PLATESHIELD_H
#define PLATESHIELD_H

#include <Arduino.h>
#ifdef ARDUINO_ARCH_AVR
#include <avr/pgmspace.h>
#endif
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

class PlateClass
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

    _rawX = analogRead(_Y1);

    return _rawX;
  }


  int getvalueY()
  {
    pinMode(_Y1, OUTPUT);
    pinMode(_Y2, OUTPUT);

    pinMode(_X1, INPUT);
    pinMode(_X2, INPUT);

    digitalWrite(_Y1, HIGH);
    digitalWrite(_Y2, LOW);

    _rawY = analogRead(_X1);

    return _rawY;
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
      _XYsetpointCircle(0) = 51.0f + 15.0f * referenceSin(indexCircle + 8);
      _XYsetpointCircle(1) = 30.0f + 15.0f * referenceSin(indexCircle);

      indexCircle++;

      if (indexCircle == 32)
      {
        indexCircle = 0;
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
      _XYsetpointOval(0) = 51.0f + 20.0f * referenceSin(indexOval + 8);
      _XYsetpointOval(1) = 30.0f + 10.0f * referenceSin(indexOval);

      indexOval++;

      if (indexOval == 32)
      {
        indexOval = 0;
      }

      lastTimeOval = millis();
    }

    return _XYsetpointOval;
  }


  // Four corner references in mm: (31,20), (71,20), (71,40), (31,40).
  // Hold each corner for 1000-1500 ms, selected by the potentiometer.
  BLA::Matrix<2, 1> rectangle(float speed)
  {
    const unsigned long interval = map(speed, 0, 1023, 1000, 1500);
    const unsigned long now = millis();

    if (!rectangleStarted)
    {
      lastTimeRectangle = now;
      rectangleStarted = true;
    }

    if (now - lastTimeRectangle >= interval)
    {
      indexRectangle = (indexRectangle + 1) % 4;
      _XYsetpointRectangle(0) =
        (indexRectangle == 1 || indexRectangle == 2) ? 71.0f : 31.0f;
      _XYsetpointRectangle(1) = indexRectangle >= 2 ? 40.0f : 20.0f;
      lastTimeRectangle = now;
    }

    return _XYsetpointRectangle;
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

  int _rawX = 0;
  int _rawY = 0;

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
  unsigned long lastTimeRectangle = 0;

  unsigned long lastTimeLQR = 0;

  float timeInterval = 0.0;


  uint8_t indexCircle = 0;
  uint8_t indexOval = 0;
  uint8_t indexRectangle = 0;
  bool rectangleStarted = false;


  // sin(2*pi*index/32), reconstructed from one quarter-wave.
  // AVR stores the shared 36-byte table in flash, not in each object's RAM.
  // Circle: (51 + 15*cos(theta), 30 + 15*sin(theta)) mm.
  // Ellipse: (51 + 20*cos(theta), 30 + 10*sin(theta)) mm.
  static float referenceSin(uint8_t index)
  {
    static const float quarterWave[9]
#ifdef ARDUINO_ARCH_AVR
      PROGMEM
#endif
      = {
        0.000000000f, 0.195090322f, 0.382683432f,
        0.555570233f, 0.707106781f, 0.831469612f,
        0.923879533f, 0.980785280f, 1.000000000f
      };

    index %= 32;
    const bool negative = index >= 16;
    uint8_t offset = index % 16;
    if (offset > 8)
    {
      offset = 16 - offset;
    }
#ifdef ARDUINO_ARCH_AVR
    const float value = pgm_read_float(&quarterWave[offset]);
#else
    const float value = quarterWave[offset];
#endif
    return negative ? -value : value;
  }


  float _Xcenter = 508;
  float _Ycenter = 482;

  float _Xmax = 948;
  float _Xmin = 89;

  float _Ymax = 842;
  float _Ymin = 137;


  BLA::Matrix<2, 1> _XY;
  BLA::Matrix<2, 1> _XYsetpointCircle = {66.0f, 30.0f};
  BLA::Matrix<2, 1> _XYsetpointOval = {71.0f, 30.0f};
  BLA::Matrix<2, 1> _XYsetpointRectangle = {31.0f, 20.0f};
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
static PlateClass PlateShield;


#endif
