#include <SamplingServo.h>
#include <BOPShield.h>
#include <PIDAbs.h>

#define KP_X 0.18
#define TI_X 0.3
#define TD_X 0.5

#define KP_Y 0.18
#define TI_Y 0.3
#define TD_Y 0.5

const float Ts = 50;   

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
  BOPShield.begin();
  BOPShield.calibration();

  PIDAbsX.setKp(KP_X);
  PIDAbsX.setTi(TI_X);
  PIDAbsX.setTd(TD_X);
  PIDAbsX.setTs(0.05);

  PIDAbsY.setKp(KP_Y);
  PIDAbsY.setTi(TI_Y);
  PIDAbsY.setTd(TD_Y);
  PIDAbsY.setTs(0.05);

  Serial.println("x, y, rX, rY, uX, uY");

  Sampling.period(Ts * 1000);
  Sampling.interrupt(stepEnable);
}

void loop() {
  if (stepFlag == true) {

    BLA::Matrix<2, 1> XY = BOPShield.sensorRead();

    x = XY(0);
    y = XY(1);

    BLA::Matrix<2, 1> XYsetpoint = BOPShield.oval(analogRead(_P));

    rX = XYsetpoint(0);
    rY = XYsetpoint(1);

    uX = PIDAbsX.compute(rX - x, -10, 10, -100, 100);
    uY = PIDAbsY.compute(rY - y, -10, 10, -100, 100);

    BOPShield.actuatorWrite(uX, uY);

    Serial.print(x); Serial.print(" ");
    Serial.print(y); Serial.print(" ");
    Serial.print(rX); Serial.print(", ");
    Serial.print(rY); Serial.print(", ");
    Serial.print(uX); Serial.print(", ");
    Serial.println(uY);
    stepFlag = false;
  }
}