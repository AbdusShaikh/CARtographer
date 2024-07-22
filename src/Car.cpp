#include "Car.h"



Car::Car(){};

Car::~Car(){};

void Car::test(){
    if (gpioInitialise() < 0){
        printf("Failed to init pigpio\n");
        exit(-1);
    }
    gpioSetMode(m_enaPin, PI_OUTPUT);
    gpioSetMode(m_LFWheelPin, PI_OUTPUT);
    gpioSetMode(m_RFWheelPin, PI_OUTPUT);

    gpioPWM(m_enaPin, m_dutyCycle);
    gpioPWM(m_enbPin, m_dutyCycle);
    // while(!ctrlCFlag){
    //     gpioWrite(m_inLeftWheelPin, 1);
    //     gpioWrite(m_rightWheelPin, 1);
    // }
    gpioWrite(m_LFWheelPin, 0);
    gpioWrite(m_RFWheelPin, 0);
    gpioTerminate();
}

int Car::init(WheelEncoderDataContainer &encoderReadings){
    if (gpioInitialise() < 0){
        printf("Failed to init pigpio\n");
        exit(-1);
    }
    gpioSetMode(m_enaPin, PI_OUTPUT);
    gpioSetMode(m_enbPin, PI_OUTPUT);
    gpioSetMode(m_LFWheelPin, PI_OUTPUT);
    gpioSetMode(m_LBWheelPin, PI_OUTPUT);
    gpioSetMode(m_RFWheelPin, PI_OUTPUT);
    gpioSetMode(m_RBWheelPin, PI_OUTPUT);
    gpioPWM(m_enaPin, m_dutyCycle);
    gpioPWM(m_enbPin, m_dutyCycle);
    gpioWrite(m_LFWheelPin, 0);
    gpioWrite(m_LBWheelPin, 0);
    gpioWrite(m_RFWheelPin, 0);
    gpioWrite(m_RBWheelPin, 0);
    m_encoderReadings = encoderReadings;
    return EXIT_SUCCESS;
}

int Car::uninit(){
    gpioWrite(m_LFWheelPin, 0);
    gpioWrite(m_LBWheelPin, 0);
    gpioWrite(m_RFWheelPin, 0);
    gpioWrite(m_RBWheelPin, 0);
    gpioTerminate();
    return EXIT_SUCCESS;
}

int Car::drive(int speed, float ang){
    ang = 0;
    gpioWrite(m_LFWheelPin, 1);
    gpioWrite(m_RFWheelPin, 1);
    // if (!m_interface->isClear){
    //     gpioWrite(m_LFWheelPin, 0);
    //     gpioWrite(m_RFWheelPin, 1);
    // }
    // else {
    //     gpioWrite(m_LFWheelPin, 1);
    //     gpioWrite(m_RFWheelPin, 1);
    // }
    return EXIT_SUCCESS;
}

void Car::main(){
    drive(125, 0.0f);
}
