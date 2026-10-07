// Rectangle: (31,20) -> (71,20) -> (71,40) -> (31,40) mm.
// The potentiometer sets the corner hold time to 1000-1500 ms.
#include <SamplingServo.h>
#include <PlateShield.h>
#include <PIDAbs.h>

#define KP_X 0.18
#define TI_X 0.3
#define TD_X 0.5

#define KP_Y 0.18
#define TI_Y 0.3
#define TD_Y 0.5

const unsigned long Ts_us = 50000UL;
const float Ts = Ts_us / 1000000.0f;  // Controller period in seconds.

float x, y;
float rX, rY;
float uX, uY;

PIDAbsClass PIDAbsX;
PIDAbsClass PIDAbsY;

volatile bool stepFlag = false;

void stepEnable() {
  stepFlag = true;
}

void setup() {
  Serial.begin(115200);
  PlateShield.begin();
  PlateShield.calibration();

  PIDAbsX.setKp(KP_X);
  PIDAbsX.setTi(TI_X);
  PIDAbsX.setTd(TD_X);
  PIDAbsX.setTs(Ts);

  PIDAbsY.setKp(KP_Y);
  PIDAbsY.setTi(TI_Y);
  PIDAbsY.setTd(TD_Y);
  PIDAbsY.setTs(Ts);

  Serial.println("x, y, rX, rY, uX, uY");

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

    BLA::Matrix<2, 1> XYsetpoint = PlateShield.rectangle(analogRead(_P));

    rX = XYsetpoint(0);
    rY = XYsetpoint(1);

    uX = PIDAbsX.compute(rX - x, -10, 10, -100, 100);
    uY = PIDAbsY.compute(rY - y, -10, 10, -100, 100);

    PlateShield.actuatorWrite(uX, uY);

    Serial.print(x); Serial.print(", ");
    Serial.print(y); Serial.print(", ");
    Serial.print(rX); Serial.print(", ");
    Serial.print(rY); Serial.print(", ");
    Serial.print(uX); Serial.print(", ");
    Serial.println(uY);
  }
}
