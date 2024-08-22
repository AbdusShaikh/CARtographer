#include <pigpio.h>
#include <thread>
#include "common.h"
#define DEBUG_CAR 0

struct carPinConfig{
    int enA = 21;
    int enB = 26;

    int lFWheel = 19;
    int lBWheel = 13;
    int rFWheel = 20;
    int rBWheel = 16;

    int leftEncoderA = 14;
    int leftEncoderB = 15;
    int rightEncoderA = 18;
    int rightEncoderB = 23;

};

struct carDimensions {
    float wheelDiameter_mm = 42.0f; // Diameter of wheels in mm
    float wheelCircumference = wheelDiameter_mm * M_PI;
    float wheelBase_mm = 90.0f;
    float encoderSlots = 12.0f;
};

class Car{
    public:
        Car();
        ~Car();

        int init(OdometryDataContainer *odometryDeposit);
        int uninit();
        int drive(int speed, float ang);
        void remoteDrive(char command);
        void computeOdometry();
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
            // int* pulses = &instance->m_encoderReadings->leftWheel;
            float* pulses = &instance->m_leftWheelTicks;
            int direction = 1; // Forward on right wheel is opposite to forward on left wheel
            string debugOutput = "[Car:Encoder Pulses] Left ";
            // Switch to right side if that is what called this function.
            if (gpio == instance->m_pinConfig.rightEncoderA){
                pinA = instance->m_pinConfig.rightEncoderA;
                pinB = instance->m_pinConfig.rightEncoderB;
                // pulses = &instance->m_encoderReadings->rightWheel;
                pulses = &instance->m_rightWheelTicks;
                direction = -1; // Forward on right wheel is opposite to forward on left wheel
                debugOutput = "[Car:Encoder Pulses] Right ";
            }
            if (gpioRead(pinA) != gpioRead(pinB)){
                (*pulses) += 1 * direction; // B edge has not risen yet. B comes after A in clockwise motion
            } else {
                (*pulses) -= 1 * direction; // B edge has already risen. B comes before A in counter clockwise motion
            }
#if DEBUG_CAR
            debugOutput += to_string(*pulses) + "\n";
            printf("%s", debugOutput.c_str());
#endif

        }
        carPinConfig m_pinConfig;
        carDimensions m_carDimensions;
        OdometryDataContainer *m_odometry;
        // TODO: Organize
        float m_dutyCycle = 80.0f;
        float m_leftWheelTicks = 0.0f; // Number of left wheel encoder ticks since last timestep
        float m_rightWheelTicks = 0.0f; // Number of right wheel encoder ticks since last timestep
};