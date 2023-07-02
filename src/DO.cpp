#include <Arduino.h>
#include <pgmspace.h>
#include <EEPROM.h>
#include "DO.h"

float doValue;
float temperature;

#define EEPROM_write(address, p)        \
  {                                     \
    int i = 0;                          \
    byte *pp = (byte *)&(p);            \
    for (; i < sizeof(p); i++)          \
      EEPROM.write(address + i, pp[i]); \
  }
#define EEPROM_read(address, p)         \
  {                                     \
    int i = 0;                          \
    byte *pp = (byte *)&(p);            \
    for (; i < sizeof(p); i++)          \
      pp[i] = EEPROM.read(address + i); \
  }

#define ReceivedBufferLength 20
char receivedBuffer[ReceivedBufferLength + 1];
byte receivedBufferIndex = 0;

#define SCOUNT 30
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;

#define SatDOVoltAddr 12
#define SatDOTempAddr 16
float SatDOVolt, SatDOTemp;
float avgVolt;

const float DO_Table[41] PROGMEM = {
14.46, 14.22, 13.82, 13.44, 13.09,
12.74, 12.42, 12.11, 11.81, 11.53,
11.26, 11.01, 10.77, 10.53, 10.30,
10.08, 9.86,  9.66,  9.46,  9.27,
9.08,  8.90,  8.73,  8.57,  8.41,
8.25,  8.11,  7.96,  7.82,  7.69,
7.56,  7.43,  7.30,  7.18,  7.07,
6.95,  6.84,  6.73,  6.63,  6.53,
6.41,
};

DO::DO(uint8_t pin, double vref, double aref)
{
  _pin = pin;
  _vref = vref;
  _aref = aref;
}

DO::~DO() {}

boolean DO::serialDataDO(void)
{
  char receivedChar;
  static unsigned long receivedTimeOut = millis();
  while (Serial.available() > 0)
  {
    if (millis() - receivedTimeOut > 500U)
    {
      receivedBufferIndex = 0;
      memset(receivedBuffer, 0, (ReceivedBufferLength + 1));
    }
    receivedTimeOut = millis();
    receivedChar = Serial.read();
    if (receivedChar == '\n' || receivedBufferIndex == ReceivedBufferLength)
    {
      receivedBufferIndex = 0;
      strupr(receivedBuffer);
      return true;
    }
    else
    {
      receivedBuffer[receivedBufferIndex] = receivedChar;
      receivedBufferIndex++;
    }
  }
  return false;
}

byte DO::uartParsingDO()
{
  byte modeIndex = 0;
  modeIndex = (strstr(receivedBuffer, "CALIBRATION") != NULL) ? 1 :
              (strstr(receivedBuffer, "SATCAL") != NULL) ? 2 :
              (strstr(receivedBuffer, "EXIT") != NULL) ? 3 : 0;
  return modeIndex;
}


void DO::calibrationDO(byte mode)
{
  char *receivedBufferPtr;
  static boolean finishCalDO = 0, enterCalMode = 0;

  switch (mode)
  {
  case 0:
    if (enterCalMode)
      Serial.println(F("Command Error"));
    break;

  case 1:
    enterCalMode = 1;
    finishCalDO = 0;
    Serial.println();
    Serial.println(F(">>>Enter Calibration Mode<<<"));
    Serial.println(F(">>>Please put the probe into the saturation oxygen water! <<<"));
    Serial.println();
    break;

  case 2:
    if (enterCalMode)
    {
      Serial.println();
      Serial.println(F(">>>Saturation Calibration Finish!<<<"));
      Serial.println();
      EEPROM_write(SatDOVoltAddr, avgVolt);
      EEPROM_write(SatDOTempAddr, temperature);
      SatDOVolt = avgVolt;
      SatDOTemp = temperature;
      finishCalDO = 1;
    }
    break;

  case 3:
    if (enterCalMode)
    {
      Serial.println();
      if (finishCalDO)
        Serial.print(F(">>>Calibration Successful"));
      else
        Serial.print(F(">>>Calibration Failed"));
      Serial.println(F(",Exit Calibration Mode<<<"));
      Serial.println();
      finishCalDO = 0;
      enterCalMode = 0;
    }
    break;
  }
}

int DO::getMedianDO(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
  {
    bTab[i] = bArray[i];
  }
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  bTemp = ((iFilterLen & 1) > 0) ? bTab[(iFilterLen - 1) / 2] : (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

void DO::characteristicDO(void)
{
  EEPROM_read(SatDOVoltAddr, SatDOVolt);
  EEPROM_read(SatDOTempAddr, SatDOTemp);
  if (EEPROM.read(SatDOVoltAddr) == 0xFF && EEPROM.read(SatDOVoltAddr + 1) == 0xFF && EEPROM.read(SatDOVoltAddr + 2) == 0xFF && EEPROM.read(SatDOVoltAddr + 3) == 0xFF)
  {
    SatDOVolt = 1127.6; // default voltage:1127.6mv
    EEPROM_write(SatDOVoltAddr, SatDOVolt);
  }
  if (EEPROM.read(SatDOTempAddr) == 0xFF && EEPROM.read(SatDOTempAddr + 1) == 0xFF && EEPROM.read(SatDOTempAddr + 2) == 0xFF && EEPROM.read(SatDOTempAddr + 3) == 0xFF)
  {
    SatDOTemp = 25.0; // default temperature is 25^C
    EEPROM_write(SatDOTempAddr, SatDOTemp);
  }
}

void DO::setTemperature(float temp)
{
    _temp = temp;
}

float DO::getTemperature()
{
    return _temp;
}

int DO::analogDO(){
  return analogRead(_pin);
}

float DO::voltageDO(){
  return analogDO() * _vref / _aref;
}

float DO::samplingVoltDO(){
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 30U) // every 30 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(_pin); // read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 1000U)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
    {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    }
    avgVolt = getMedianDO(analogBufferTemp, SCOUNT) * (float)_vref / _aref;
  }
  return avgVolt;
}

float DO::samplingTempDO(){
  static unsigned long tempSampleTimepoint = millis();
  if (millis() - tempSampleTimepoint > 500U) // every 500 milliseconds, read the temperature
  {
    //tempSampleTimepoint = millis();
    temperature = getTemperature();  // add your temperature codes here to read the temperature, unit:^C
  }
  return temperature;
}

float DO::getDO(){
  doValue = pgm_read_float_near(&DO_Table[0] + (int)(SatDOTemp + 0.5)) * samplingVoltDO() / SatDOVolt;
  return doValue;
}

void DO::modeDO(){
  if (serialDataDO() > 0)
  {
    byte modeIndex = uartParsingDO();
    calibrationDO(modeIndex);
  }
  EEPROM.commit();
}

void DO::getAllDOData(){
  Serial.print("Temp: " + String(getTemperature()) + "°C | ");
  Serial.print("DO Val: " + String(getDO()) + " mg/L | ");
  Serial.print("Sat Volt: " + String(SatDOVolt) + " mV | ");
  Serial.print("Avg Volt: " + String(samplingVoltDO()) + " mV | ");
  Serial.print("Volt in: " + String(voltageDO()) + " mV | ");
  Serial.println("Analog in: " + String(analogDO()) + " | ");
}

void DO::begin()
{
  EEPROM.begin(512);
  pinMode(_pin, INPUT);
  characteristicDO();
}

void DO::run()
{
  samplingTempDO();
  samplingVoltDO();
  modeDO();
}