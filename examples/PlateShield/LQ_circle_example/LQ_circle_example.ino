#include <SamplingServo.h>
#include <PlateShield.h>


const unsigned long Ts_us = 50000UL;
const float Ts = Ts_us / 1000000.0f;  // Controller period in seconds.

float rX;
float rY;

float x = 0.0;
float y = 0.0;

float xPrev = 0.0;
float yPrev = 0.0;

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

  Serial.println("x, y, rX, rY, dx, dy, intX, intY, uX, uY");

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

    dx = (x - xPrev) / Ts;
    dy = (y - yPrev) / Ts;

    float eX = rX - x;
    float eY = rY - y;

    intX = intX + Ts * eX;
    intY = intY + Ts * eY;

    intX = constrain(intX, -100.0, 100.0);
    intY = constrain(intY, -100.0, 100.0);

    BLA::Matrix<6, 1> Xa = {
      x - rX,
      dx,
      y - rY,
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
    Serial.print(uX);      Serial.print(", ");
    Serial.println(uY);
  }
}
