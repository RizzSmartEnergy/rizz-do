#ifndef DO_H
#define DO_H
#include <Arduino.h>

class DO
{
public:
    DO(uint8_t pin, double vref, double aref);
    ~DO();
    boolean serialDataDO(void);
    byte uartParsingDO();
    void calibrationDO(byte mode);
    int getMedianDO(int bArray[], int iFilterLen);
    void characteristicDO(void);
    void setTemperature(float temp);
    float getTemperature();
    int analogDO();
    float voltageDO();
    float samplingVoltDO();
    float samplingTempDO();
    float getDO();
    void modeDO();
    void getAllDOData();
    void begin();
    void run();

private:
    uint8_t _pin;
    double _vref, _aref;
    float _temp;
    // int _baudrate, _delay_time;
};

#endif