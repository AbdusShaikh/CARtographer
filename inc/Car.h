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
        int in1Pin = 20;
        int m_in2Pin = 16;
        int m_dutyCycle = 255;

        iFace* m_interface;
};