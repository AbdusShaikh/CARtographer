#include "Car.h"



Car::Car(){};

Car::~Car(){};

void Car::test(){
    if (gpioInitialise() < 0){
        printf("Failed to init pigpio\n");
        exit(-1);
    }
    gpioSetMode(m_enaPin, PI_OUTPUT);
    gpioSetMode(in1Pin, PI_OUTPUT);
    gpioSetMode(m_in2Pin, PI_OUTPUT);

    gpioPWM(m_enaPin, m_dutyCycle);
    // while(!ctrlCFlag){
    //     gpioWrite(in1Pin, 1);
    //     gpioWrite(m_in2Pin, 1);
    // }
    gpioWrite(in1Pin, 0);
    gpioWrite(m_in2Pin, 0);
    gpioTerminate();
}

int Car::init(iFace* interface){
    if (gpioInitialise() < 0){
        printf("Failed to init pigpio\n");
        exit(-1);
    }
    gpioSetMode(m_enaPin, PI_OUTPUT);
    gpioSetMode(in1Pin, PI_OUTPUT);
    gpioSetMode(m_in2Pin, PI_OUTPUT);
    gpioPWM(m_enaPin, m_dutyCycle);
    gpioWrite(in1Pin, 0);
    gpioWrite(m_in2Pin, 0);
    m_interface = interface;
    return 0;
}

int Car::uninit(){
    gpioWrite(in1Pin, 0);
    gpioWrite(m_in2Pin, 0);
    gpioTerminate();
    return 1;
}

int Car::drive(int speed, float ang){
    gpioPWM(m_enaPin, speed);
    ang = 0;
    if (!m_interface->isClear){
        gpioWrite(in1Pin, 0);
        gpioWrite(m_in2Pin, 0);
    }
    else {
        gpioWrite(in1Pin, 1);
        gpioWrite(m_in2Pin, 1);
    }
    return 0;
}

void Car::main(){
    drive(255, 0.0f);
}
