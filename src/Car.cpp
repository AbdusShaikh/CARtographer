#include "Car.h"



Car::Car(){};

Car::~Car(){};

void Car::test(){
    if (gpioInitialise() < 0){
        printf("Failed to init pigpio\n");
        exit(-1);
    }
    gpioSetMode(m_enaPin, PI_OUTPUT);
    gpioSetMode(m_inLeftWheelPin, PI_OUTPUT);
    gpioSetMode(m_rightWheelPin, PI_OUTPUT);

    gpioPWM(m_enaPin, m_dutyCycle);
    // while(!ctrlCFlag){
    //     gpioWrite(m_inLeftWheelPin, 1);
    //     gpioWrite(m_rightWheelPin, 1);
    // }
    gpioWrite(m_inLeftWheelPin, 0);
    gpioWrite(m_rightWheelPin, 0);
    gpioTerminate();
}

int Car::init(iFace* interface){
    if (gpioInitialise() < 0){
        printf("Failed to init pigpio\n");
        exit(-1);
    }
    gpioSetMode(m_enaPin, PI_OUTPUT);
    gpioSetMode(m_inLeftWheelPin, PI_OUTPUT);
    gpioSetMode(m_rightWheelPin, PI_OUTPUT);
    gpioPWM(m_enaPin, m_dutyCycle);
    gpioWrite(m_inLeftWheelPin, 0);
    gpioWrite(m_rightWheelPin, 0);
    m_interface = interface;
    return 0;
}

int Car::uninit(){
    gpioWrite(m_inLeftWheelPin, 0);
    gpioWrite(m_rightWheelPin, 0);
    gpioTerminate();
    return 1;
}

int Car::drive(int speed, float ang){
    gpioPWM(m_enaPin, speed);
    ang = 0;
    if (!m_interface->isClear){
        gpioWrite(m_inLeftWheelPin, 0);
        gpioWrite(m_rightWheelPin, 1);
    }
    else {
        gpioWrite(m_inLeftWheelPin, 1);
        gpioWrite(m_rightWheelPin, 1);
    }
    return 0;
}

void Car::main(){
    drive(255, 0.0f);
}
