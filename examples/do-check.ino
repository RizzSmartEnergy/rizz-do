#include <Arduino.h>
#include <DO.h>

#define SENSOR_PIN 34
#define VREF 3300
#define AREF 4095
#define CAL_MODE 1

DO sensor(SENSOR_PIN, VREF, AREF);

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    sensor.setTemperature(25);
    sensor.getAllDOData(1000);
}