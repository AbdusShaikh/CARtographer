#include <pigpio.h>
#include "common.h"


class Car{
    public:
        Car();
        ~Car();

        int init(WheelEncoderDataContainer &encoderReadings);
        int uninit();
        int drive(int speed, float ang);
        void test();
        void main();
    private:
        int m_enaPin = 21; // Right
        int m_enbPin = 26; // Left
        int m_LFWheelPin= 19;
        int m_LBWheelPin = 13;
        int m_RFWheelPin = 20; 
        int m_RBWheelPin = 16; 
        int m_dutyCycle = 255;

        WheelEncoderDataContainer m_encoderReadings;
};