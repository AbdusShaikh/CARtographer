#include <pigpio.h>
#include "common.h"

class Car{
    public:
        Car();
        void test();
    private:
        uint8_t m_enaPin = 21;
        uint8_t in1Pin = 20;
        uint8_t m_in2Pin = 16;
        uint8_t m_dutyCycle = 255;
};