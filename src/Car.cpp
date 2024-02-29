#include "Car.h"

volatile sig_atomic_t ctrlCFlag = 0;
void sigintHandler(int sig_num){
    ctrlCFlag = 1;
}

Car::Car(){};


void Car::test(){
    if (gpioInitialise() < 0){
        printf("Failed to init pigpio\n");
        exit(-1);
    }
    signal(SIGINT, sigintHandler);
    gpioSetMode(m_enaPin, PI_OUTPUT);
    gpioSetMode(in1Pin, PI_OUTPUT);
    gpioSetMode(m_in2Pin, PI_OUTPUT);

    gpioPWM(m_enaPin, m_dutyCycle);
    while(!ctrlCFlag){
        gpioWrite(in1Pin, 1);
        gpioWrite(m_in2Pin, 1);
    }
    gpioWrite(in1Pin, 0);
    gpioWrite(m_in2Pin, 0);
    gpioTerminate();
}