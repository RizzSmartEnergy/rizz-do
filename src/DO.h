#ifndef DO_H
#define DO_H
#include <Arduino.h>

class DO
{
public:
    DO(uint8_t pin, double vref, double aref, int cal_mode);
    ~DO();
    void begin(int baudrate);
    void setTemperature(float temp);
    float getTemperature();
    float getAnalogDO();
    float getVoltageDO();
    int getMedianDO(int bArray[], int iFilterLen);
    float samplingDO();
    float getDO();
    void getAllDOData(int delay_time);

private:
    uint8_t _pin;
    double _vref, _aref;
    float _temp;
    int _cal_mode, _baudrate, _delay_time;
};

#endif