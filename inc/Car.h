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
        int m_enbPin = 26;
        int m_leftWheelForwardPin= 19;
        int m_leftWheelBackwardPin = 13;
        int m_rightWheelBackwardPin = 20; 
        int m_leftWheelBackwardPin = 16; 
        int m_dutyCycle = 255;

        iFace* m_interface;
};