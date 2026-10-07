/*
  PlateShield LQI circle tracking with selectable state estimation.
  USE_KALMAN = 0: measured position and backward difference velocity.
  USE_KALMAN = 1: Kalman estimates of position and velocity on each axis.
  Positions: mm; velocities: mm/s; actuator commands: degrees.
  The potentiometer controls the circle reference timing.
  Controller gains and 50 ms sampling match LQ_circle_example.
*/
#define USE_KALMAN 0

#if USE_KALMAN != 0 && USE_KALMAN != 1
#error "USE_KALMAN must be 0 or 1"
#endif

#include <SamplingServo.h>
#include <PlateShield.h>
#if USE_KALMAN
// Starting values to tune on hardware, not identified noise covariances.
// Larger Q_ACCEL follows acceleration faster; larger R trusts measurements less.
const float Q_ACCEL_X = 10000.0f;  // Acceleration variance (mm/s^2)^2.
const float Q_ACCEL_Y = 10000.0f;
const float R_POSITION_X = 1.0f;   // Measurement variance mm^2; must be > 0.
const float R_POSITION_Y = 1.0f;
#endif


const unsigned long Ts_us = 50000UL;
const float Ts = Ts_us / 1000000.0f;  // Controller period in seconds.

#if USE_KALMAN
// One joint filter avoids sharing the library's static estimate between axes.
// State: [x, vx, y, vy]. Constant velocity model, acceleration as process noise.
// No identified actuator model: B is zero and the filter input is zero.
BLA::Matrix<4, 4> A = {
  1.0f, Ts,   0.0f, 0.0f,
  0.0f, 1.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 1.0f, Ts,
  0.0f, 0.0f, 0.0f, 1.0f
};
BLA::Matrix<4, 1> B = {0.0f, 0.0f, 0.0f, 0.0f};
BLA::Matrix<2, 4> C = {
  1.0f, 0.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 1.0f, 0.0f
};
const float qPosition = 0.25f * Ts * Ts * Ts * Ts;
const float qCross = 0.5f * Ts * Ts * Ts;
const float qVelocity = Ts * Ts;
BLA::Matrix<4, 4> Q_Kalman = {
  qPosition * Q_ACCEL_X, qCross * Q_ACCEL_X, 0.0f, 0.0f,
  qCross * Q_ACCEL_X, qVelocity * Q_ACCEL_X, 0.0f, 0.0f,
  0.0f, 0.0f, qPosition * Q_ACCEL_Y, qCross * Q_ACCEL_Y,
  0.0f, 0.0f, qCross * Q_ACCEL_Y, qVelocity * Q_ACCEL_Y
};
BLA::Matrix<2, 2> R_Kalman = {R_POSITION_X, 0.0f, 0.0f, R_POSITION_Y};
BLA::Matrix<4, 1> xIC = {0.0f, 0.0f, 0.0f, 0.0f};
BLA::Matrix<4, 1> X = {0.0f, 0.0f, 0.0f, 0.0f};
#endif

float rX;
float rY;

float x = 0.0;
float y = 0.0;

float xPrev = 0.0;
float yPrev = 0.0;

float xEstimate = 0.0;
float yEstimate = 0.0;

float dx = 0.0;
float dy = 0.0;

float intX = 0.0;
float intY = 0.0;

float uX = 0.0;
float uY = 0.0;

volatile bool stepFlag = false;

BLA::Matrix<2, 6> K = {
  -0.43,  -0.11,   0.0,    0.0,    0.2,   0.0,
   0.0,    0.0,   -0.35,  -0.1,    0.0,   0.13
};

void stepEnable() {
  stepFlag = true;
}

void setup() {
  Serial.begin(115200);
  PlateShield.begin();
  PlateShield.calibration();

  BLA::Matrix<2, 1> XY = PlateShield.sensorRead();

  x = XY(0);
  y = XY(1);

  xPrev = x;
  yPrev = y;

#if USE_KALMAN
  xIC(0) = x;
  xIC(2) = y;
#endif

  Serial.println(F("x, y, rX, rY, xEstimate, yEstimate, dx, dy, uX, uY"));

  Sampling.period(Ts_us);
  Sampling.interrupt(stepEnable);
}

void loop() {
  // Consume the timer flag atomically; run control and Serial outside the ISR.
  noInterrupts();
  const bool runStep = stepFlag;
  stepFlag = false;
  interrupts();

  if (runStep) {

    BLA::Matrix<2, 1> XY = PlateShield.sensorRead();

    x = XY(0);
    y = XY(1);

    BLA::Matrix<2, 1> XYsetpoint = PlateShield.circle(analogRead(_P));

    rX = XYsetpoint(0);
    rY = XYsetpoint(1);

#if USE_KALMAN
    BLA::Matrix<2, 1> Y = {x, y};
    PlateShield.getKalmanEstimate(X, 0.0f, Y, A, B, C, Q_Kalman, R_Kalman, xIC);
    xEstimate = X(0);
    dx = X(1);
    yEstimate = X(2);
    dy = X(3);
#else
    xEstimate = x;
    yEstimate = y;
    dx = (x - xPrev) / Ts;
    dy = (y - yPrev) / Ts;
#endif

    float eX = rX - xEstimate;
    float eY = rY - yEstimate;

    intX = intX + Ts * eX;
    intY = intY + Ts * eY;

    intX = constrain(intX, -100.0, 100.0);
    intY = constrain(intY, -100.0, 100.0);

    BLA::Matrix<6, 1> Xa = {
      xEstimate - rX,
      dx,
      yEstimate - rY,
      dy,
      intX,
      intY
    };

    BLA::Matrix<2, 1> U = K * Xa;

    uX = U(0);
    uY = U(1);

    uX = constrain(uX, -10.0, 10.0);
    uY = constrain(uY, -10.0, 10.0);

    PlateShield.actuatorWrite(uX, uY);

    xPrev = x;
    yPrev = y;

    Serial.print(x);       Serial.print(", ");
    Serial.print(y);       Serial.print(", ");
    Serial.print(rX);      Serial.print(", ");
    Serial.print(rY);      Serial.print(", ");
    Serial.print(xEstimate); Serial.print(", ");
    Serial.print(yEstimate); Serial.print(", ");
    Serial.print(dx);      Serial.print(", ");
    Serial.print(dy);      Serial.print(", ");
    Serial.print(uX);      Serial.print(", ");
    Serial.println(uY);
  }
}
