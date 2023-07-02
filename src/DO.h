#ifndef DO_H
#define DO_H
#include <Arduino.h>

class DO
{
public:
    DO(uint8_t pin, double vref, double aref);
    ~DO();
    void begin(int baudrate);
    void characteristicDO();
    boolean serialDataDO();
    byte uartParsingDO();
    void calibrationDO(byte mode);
    void setTemperature(float temp);
    float getTemperature();
    float getAnalogDO();
    float getVoltageDO();
    int getMedianDO(int bArray[], int iFilterLen);
    float samplingVoltageDO();
    float samplingTempDO();
    float getDO();
    void run();
    void modeDO();
    void getAllDOData(int delay_time);

private:
    uint8_t _pin;
    double _vref, _aref;
    float _temp;
    int _baudrate, _delay_time;
};

#endif