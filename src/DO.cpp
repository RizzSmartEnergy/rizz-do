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

void DO::setTemperature(uint16_t temp)
{
    _temp = temp;
}

uint16_t DO::getTemperature()
{
    return _temp;
}

uint16_t DO::analogDO()
{
    return analogRead(_pin);
}

float DO::voltageDO()
{
    return (_vref * analogRead(_pin) / _aref);
}

static const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

int16_t DO::getDOValue()
{
    uint16_t V_saturation = (_cal_mode == 1) ? ((uint32_t)CAL1_V + (uint32_t)35 * _temp - (uint32_t)CAL1_T * 35) : ((int16_t)((int8_t)_temp - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V);

    return ((voltageDO() * DO_Table[_temp] / V_saturation) / 1000);
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
