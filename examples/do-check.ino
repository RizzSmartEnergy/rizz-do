#include <Arduino.h>
#include <DO.h>

#define PIN 34
#define AREF 4095
#define VREF 3300

DO sensor(PIN, VREF, AREF);

void setup(){
  Serial.begin(115200);
  sensor.begin();
}

void loop(){
  sensor.setTemperature(25.0);
  sensor.run();
  sensor.getAllDOData();
  delay(500);
}