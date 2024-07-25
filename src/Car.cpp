#include "Car.h"



Car::Car(){};

Car::~Car(){};

void Car::test(){
    if (gpioInitialise() < 0){
        printf("Failed to init pigpio\n");
        exit(-1);
    }
    gpioSetMode(m_pinConfig.enA, PI_OUTPUT);
    gpioSetMode(m_pinConfig.lFWheel, PI_OUTPUT);
    gpioSetMode(m_pinConfig.rFWheel, PI_OUTPUT);

    gpioPWM(m_pinConfig.enA, m_dutyCycle);
    gpioPWM(m_pinConfig.enB, m_dutyCycle);
    // while(!ctrlCFlag){
    //     gpioWrite(m_inLeftWheelPin, 1);
    //     gpioWrite(m_rightWheelPin, 1);
    // }
    gpioWrite(m_pinConfig.lFWheel, 0);
    gpioWrite(m_pinConfig.rFWheel, 0);
    gpioTerminate();
}

int Car::init(WheelEncoderDataContainer &encoderReadings){
    if (gpioInitialise() < 0){
        printf("Failed to init pigpio\n");
        exit(-1);
    }
    // Set pin values
    m_pinConfig.enA = 21;
    m_pinConfig.enB = 26;
    m_pinConfig.lFWheel = 19;
    m_pinConfig.lBWheel = 13;
    m_pinConfig.rFWheel = 20;
    m_pinConfig.rBWheel = 16;

    // Initialize pins
    gpioSetMode(m_pinConfig.enA, PI_OUTPUT);
    gpioSetMode(m_pinConfig.enB, PI_OUTPUT);
    gpioSetMode(m_pinConfig.lFWheel, PI_OUTPUT);
    gpioSetMode(m_pinConfig.lBWheel, PI_OUTPUT);
    gpioSetMode(m_pinConfig.rFWheel, PI_OUTPUT);
    gpioSetMode(m_pinConfig.rBWheel, PI_OUTPUT);
    gpioPWM(m_pinConfig.enA, m_dutyCycle);
    gpioPWM(m_pinConfig.enB, m_dutyCycle);
    gpioWrite(m_pinConfig.lFWheel, 0);
    gpioWrite(m_pinConfig.lBWheel, 0);
    gpioWrite(m_pinConfig.rFWheel, 0);
    gpioWrite(m_pinConfig.rBWheel, 0);

    // Read encoders
    gpioSetAlertFuncEx(m_pinConfig.leftEncoderA, readEncoders, this); 
    gpioSetAlertFuncEx(m_pinConfig.rightEncoderA, readEncoders, this);
    m_encoderReadings = encoderReadings;
    return EXIT_SUCCESS;
}

int Car::uninit(){
    gpioWrite(m_pinConfig.lFWheel, 0);
    gpioWrite(m_pinConfig.lBWheel, 0);
    gpioWrite(m_pinConfig.rFWheel, 0);
    gpioWrite(m_pinConfig.rBWheel, 0);
    gpioTerminate();
    return EXIT_SUCCESS;
}

int Car::drive(int speed, float ang){
    (void) ang; // REMOVE
    (void) speed; // REMOVE
    gpioWrite(m_pinConfig.lFWheel, 1);
    gpioWrite(m_pinConfig.rFWheel, 1);
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

// void Car::readEncoders(){
//     if (gpioRead(m_pinConfig.leftEncoderA) != gpioRead(m_pinConfig.leftEncoderB)){
//         m_encoderReadings.leftWheel += 1;
//     }
//     else {
//         m_encoderReadings.leftWheel -=1 ;
//     }
// }

void Car::main(){
    drive(125, 0.0f);
}
