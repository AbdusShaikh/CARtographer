#include <pigpio.h>
#include <thread>
#include "common.h"

struct carPinConfig{
    int enA;
    int enB;

    int lFWheel;
    int lBWheel;
    int rFWheel;
    int rBWheel;

    int leftEncoderA;
    int leftEncoderB;
    int rightEncoderA;
    int rightEncoderB;

};

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
        // Callback function to be executed upon recieving a rising edge on motor encoder A from either left or right side.
        static void readEncoders(int gpio, int level, uint32_t tick, void* userData){
            if (level != 1){ // If not a change to a rising edge
                return;
            }
            (void) tick; // REMOVE
            Car* instance = (Car*) userData;
            // Default to left side
            int pinA = instance->m_pinConfig.leftEncoderA;
            int pinB = instance->m_pinConfig.leftEncoderB;
            int* pulses = &instance->m_encoderReadings.leftWheel;
            // Switch to right side if that is what called this function.
            if (gpio == instance->m_pinConfig.rightEncoderA){
                pinA = instance->m_pinConfig.rightEncoderA;
                pinB = instance->m_pinConfig.rightEncoderB;
                pulses = &instance->m_encoderReadings.rightWheel;
            }
            if (gpioRead(pinA) != gpioRead(pinB)){
                (*pulses)++; // B edge has not risen yet. B comes after A in clockwise motion
            } else {
                (*pulses) --; // B edge has already risen. B comes before A in counter clockwise motion
            }
        }
        carPinConfig m_pinConfig;
        int m_dutyCycle = 255;

        WheelEncoderDataContainer m_encoderReadings;
};