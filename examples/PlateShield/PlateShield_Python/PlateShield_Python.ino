/*
  PlateShield USB serial bridge, 115200 baud, newline-delimited ASCII.
  Upload once, then close Serial Monitor before connecting from Python.
  HELLO -> PLATESHIELD 1
  READ / SET <uX> <uY> / STOP -> DATA <millis> <x_mm> <y_mm> <uX> <uY>
  Commands are offsets in degrees, limited to [-10, 10].
  STOP and 500 ms without a valid SET return to actuatorWrite(0, 0).
  This holds the nominal neutral servo positions; it does not detach servos.
*/
#include <PlateShield.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

const unsigned long COMMAND_TIMEOUT_MS = 500;
char command[64];
uint8_t commandLength = 0;
bool discardLine = false;
bool active = false;
unsigned long lastCommand = 0;
float uX = 0.0f;
float uY = 0.0f;

void neutral() {
  uX = 0.0f;
  uY = 0.0f;
  active = false;
  PlateShield.actuatorWrite(uX, uY);
}

void sendMeasurement() {
  BLA::Matrix<2, 1> xy = PlateShield.sensorRead();
  Serial.print(F("DATA "));
  Serial.print(millis()); Serial.print(' ');
  Serial.print(xy(0), 3); Serial.print(' ');
  Serial.print(xy(1), 3); Serial.print(' ');
  Serial.print(uX, 3); Serial.print(' ');
  Serial.println(uY, 3);
}

bool parseNumber(char *token, float &value) {
  if (!token) return false;
  char *end;
  value = strtod(token, &end);
  return end != token && *end == '\0' && isfinite(value);
}

void processCommand() {
  char *operation = strtok(command, " \t");
  if (!operation) return;
  char *first = strtok(NULL, " \t");
  if (strcmp(operation, "SET") == 0) {
    char *second = strtok(NULL, " \t");
    char *extra = strtok(NULL, " \t");
    float requestedX, requestedY;
    if (extra || !parseNumber(first, requestedX) || !parseNumber(second, requestedY)
        || fabs(requestedX) > 10.0f || fabs(requestedY) > 10.0f) {
      neutral();
      Serial.println(F("ERR SET_RANGE_OR_FORMAT"));
      return;
    }
    uX = requestedX;
    uY = requestedY;
    PlateShield.actuatorWrite(uX, uY);
    lastCommand = millis();
    active = true;
    sendMeasurement();
  } else if (!first && strcmp(operation, "HELLO") == 0) {
    neutral();
    Serial.println(F("PLATESHIELD 1"));
  } else if (!first && strcmp(operation, "READ") == 0) {
    sendMeasurement();
  } else if (!first && strcmp(operation, "STOP") == 0) {
    neutral();
    sendMeasurement();
  } else {
    neutral();
    Serial.println(F("ERR COMMAND"));
  }
}

void setup() {
  Serial.begin(115200);
  PlateShield.begin();
  neutral();
}

void loop() {
  if (active && millis() - lastCommand >= COMMAND_TIMEOUT_MS) neutral();

  // Consume one byte per loop so incomplete input cannot block the watchdog.
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      if (discardLine) {
        neutral();
        Serial.println(F("ERR LINE"));
      } else {
        command[commandLength] = '\0';
        processCommand();
      }
      commandLength = 0;
      discardLine = false;
    } else if (c != '\r' && !discardLine) {
      if (c == '\0' || commandLength >= sizeof(command) - 1) {
        discardLine = true;
        neutral();
      } else {
        command[commandLength++] = c;
      }
    }
  }
}
