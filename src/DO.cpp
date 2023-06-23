#include <Arduino.h>
#include "DO.h"

#define CAL1_V (131)
#define CAL1_T (25)

#define CAL2_V (1300)
#define CAL2_T (15)

#define SCOUNT 30

int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
float averageVoltage = 0;

DO::DO(uint8_t pin, double vref, double aref, int cal_mode)
{
    _pin = pin;
    _vref = vref;
    _aref = aref;
    _cal_mode = cal_mode;
}

DO::~DO() {}

void DO::begin(int baudrate)
{
    _baudrate = baudrate;
    Serial.begin(_baudrate);
    pinMode(_pin, INPUT);
}

void DO::setTemperature(float temp)
{
    _temp = temp;
}

float DO::getTemperature()
{
    return _temp;
}

float DO::getAnalogDO()
{
    return analogRead(_pin);
}

float DO::getVoltageDO()
{
    return (_vref * analogRead(_pin) / _aref);
}

static const float DO_Table[41] = {
    14.460, 14.220, 13.820, 13.440, 13.090, 12.740, 12.420, 12.110, 11.810, 11.530,
    11.260, 11.010, 10.770, 10.530, 10.300, 10.080, 9.860, 9.660, 9.460, 9.270,
    9.080, 8.900, 8.730, 8.570, 8.410, 8.250, 8.110, 7.960, 7.820, 7.690,
    7.560, 7.430, 7.300, 7.180, 7.070, 6.950, 6.840, 6.730, 6.630, 6.530, 6.410};

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

float DO::samplingDO()
{
    static unsigned long analogSampleTimepoint = millis();
    if (millis() - analogSampleTimepoint > 30U) // every 30 milliseconds,read the analog value from the ADC
    {
        analogSampleTimepoint = millis();
        analogBuffer[analogBufferIndex] = getAnalogDO(); // read the analog value and store into the buffer
        analogBufferIndex++;
        if (analogBufferIndex == SCOUNT)
            analogBufferIndex = 0;
    }

    static unsigned long printTimepoint = millis();
    if (millis() - printTimepoint > 800U)
    {
        printTimepoint = millis();
        for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
        {
            analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
            averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * _vref / _aref;
        }
    }
    return averageVoltage;
}

float DO::getDO()
{
    float saturationVoltage = (_cal_mode == 1) ? ((uint32_t)CAL1_V + (uint32_t)35 * _temp - (uint32_t)CAL1_T * 35) : ((int16_t)((int8_t)_temp - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V);

    return ((samplingDO() * DO_Table[(int)_temp] / saturationVoltage));
}

void DO::getAllDOData(int delay_time)
{
    _delay_time = delay_time;
    Serial.print("Temperature:\t" + String(getTemperature()) + "\t");
    Serial.print("Input Analog:\t" + String(getAnalogDO()) + "\t");
    Serial.print("Input Voltage:\t" + String(getVoltageDO()) + "\t");
    Serial.println("DO:\t" + String(getDO()) + " mg/L");
    delay(_delay_time);
}
