#include <Arduino.h>
#include <pgmspace.h>
#include <DO.h>

#define PIN 34
#define VREF 3300
#define AREF 4095.0

DO sensor(PIN, VREF, AREF);

void setup(){
  sensor.begin();
}

void loop(){
  sensor.run();
}