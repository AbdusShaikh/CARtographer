#include <pigpio.h>
#include "common.h"

class Car{
    public:
        Car();
        ~Car();

        int init(iFace* interface);
        int uninit();
        int drive(int speed, float ang);
        void test();
        void main();
    private:
        int m_enaPin = 21;
        int m_inLeftWheelPin = 20; // in1
        int m_rightWheelPin = 16; // in2
        int m_dutyCycle = 255;

        iFace* m_interface;
};