#ifndef DO_H
#define DO_H
#include <Arduino.h>

class DO{
    public:
        DO(uint8_t pin, uint32_t vref, uint32_t aref, int cal_mode);
        ~DO();
        void setTemperature(uint16_t temp);
        int16_t getDOValue();
        uint16_t getTemperature();
        uint16_t analogDO();
        float voltageDO();
        void outputDOData(int delay_time);
    private:
        uint32_t _vref, _aref;
        uint16_t _temp;
        uint8_t _pin;
        int _cal_mode, _delay_time;
};

#endif