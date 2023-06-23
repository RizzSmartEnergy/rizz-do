#ifndef DO_H
#define DO_H
#include <Arduino.h>

class DO{
    public:
        DO(uint8_t pin, uint32_t vref, uint32_t aref, int cal_mode);
        ~DO();
        void setTemperature(float temp);
        float getTemperature();
        float analogDO();
        float voltageDO();
        float getDOValue();
        void outputDOData(int delay_time);
    private:
        uint32_t _vref, _aref;
        float _temp;
        uint8_t _pin;
        int _cal_mode, _delay_time;
};

#endif