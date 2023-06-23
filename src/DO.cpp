#include <Arduino.h>
#include "DO.h"

#define CAL1_V (131)
#define CAL1_T (25)

#define CAL2_V (1300)
#define CAL2_T (15)

DO::DO(uint8_t pin, uint32_t vref, uint32_t aref, int cal_mode)
{
    _pin = pin;
    _vref = vref;
    _aref = aref;
    _cal_mode = cal_mode;
}

DO::~DO() {}

void DO::setTemperature(float temp)
{
    _temp = temp;
}

float DO::getTemperature()
{
    return _temp;
}

float DO::analogDO()
{
    return analogRead(_pin);
}

float DO::voltageDO()
{
    return (_vref * analogRead(_pin) / _aref);
}

static const float DO_Table[41] = {
    14.460, 14.220, 13.820, 13.440, 13.090, 12.740, 12.420, 12.110, 11.810, 11.530,
    11.260, 11.010, 10.770, 10.530, 10.300, 10.080, 9.860, 9.660, 9.460, 9.270,
    9.080, 8.900, 8.730, 8.570, 8.410, 8.250, 8.110, 7.960, 7.820, 7.690,
    7.560, 7.430, 7.300, 7.180, 7.070, 6.950, 6.840, 6.730, 6.630, 6.530, 6.410};

float DO::getDOValue()
{
    float V_saturation = (_cal_mode == 1) ? ((uint32_t)CAL1_V + (uint32_t)35 * _temp - (uint32_t)CAL1_T * 35) : ((int16_t)((int8_t)_temp - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V);

    return ((voltageDO() * DO_Table[(int)_temp] / V_saturation));
}

void DO::outputDOData(int delay_time)
{
    _delay_time = delay_time;
    Serial.print("Temperaturet:\t" + String(getTemperature()) + "\t");
    Serial.print("ADC RAW:\t" + String(analogDO()) + "\t");
    Serial.print("ADC Voltage:\t" + String(voltageDO()) + "\t");
    Serial.println("DO:\t" + String(getDOValue()) + "\t");
    delay(_delay_time);
}
