#include "Car.h"
// TODO:
//  - Synchronize accecss to m_leftWheelTicks and m_rightWheelTicks


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

int Car::init(OdometryDataContainer *odometryDeposit){
    if (gpioInitialise() < 0){
        printf("Failed to init pigpio\n");
        return EXIT_FAILURE;
    }
    // Initialize pins
    gpioSetMode(m_pinConfig.enA, PI_OUTPUT);
    gpioSetMode(m_pinConfig.enB, PI_OUTPUT);
    gpioSetMode(m_pinConfig.lFWheel, PI_OUTPUT);
    gpioSetMode(m_pinConfig.lBWheel, PI_OUTPUT);
    gpioSetMode(m_pinConfig.rFWheel, PI_OUTPUT);
    gpioSetMode(m_pinConfig.rBWheel, PI_OUTPUT);
    gpioSetMode(m_pinConfig.leftEncoderA, PI_INPUT);
    gpioSetMode(m_pinConfig.leftEncoderB, PI_INPUT);
    gpioSetMode(m_pinConfig.rightEncoderA, PI_INPUT);
    gpioSetMode(m_pinConfig.rightEncoderB, PI_INPUT);

    gpioPWM(m_pinConfig.enA, m_dutyCycle);
    gpioPWM(m_pinConfig.enB, m_dutyCycle);
    gpioWrite(m_pinConfig.lFWheel, 0);
    gpioWrite(m_pinConfig.lBWheel, 0);
    gpioWrite(m_pinConfig.rFWheel, 0);
    gpioWrite(m_pinConfig.rBWheel, 0);

    // Read encoders
    gpioSetAlertFuncEx(m_pinConfig.leftEncoderA, readEncoders, this); 
    gpioSetAlertFuncEx(m_pinConfig.rightEncoderA, readEncoders, this);
    m_odometry = odometryDeposit;
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

// Vehicle odometry calculations based on slide 13 from this presentation
// https://courses.edx.org/asset-v1:ETHx+AMRx+2T2020+type@asset+block/AMR_Chli_SLAM_aSLAMproblem.pdf
void Car::computeOdometry(){
    int lTicks = m_leftWheelTicks;
    m_leftWheelTicks = 0;
    int rTicks = m_rightWheelTicks;
    m_rightWheelTicks = 0;

    float lWheelDistTravelled_mm = (lTicks / m_carDimensions.encoderSlots) * m_carDimensions.wheelCircumference;
    float rWheelDistTravelled_mm = (rTicks / m_carDimensions.encoderSlots) * m_carDimensions.wheelCircumference;

    m_odometry->carWheelBase = m_carDimensions.wheelBase_mm;
    m_odometry->leftWheelDist = lWheelDistTravelled_mm;
    m_odometry->rightWheelDist = rWheelDistTravelled_mm;

    float avgDistTravelled_mm = (lWheelDistTravelled_mm + rWheelDistTravelled_mm) / 2.0f;
    float wheelDiff_mm = ((rWheelDistTravelled_mm - lWheelDistTravelled_mm));

    float dx_mm = (avgDistTravelled_mm * cos(m_odometry->globalTheta + (wheelDiff_mm / (2.0f * m_carDimensions.wheelBase_mm))));
    float dy_mm = (avgDistTravelled_mm * sin(m_odometry->globalTheta + (wheelDiff_mm / (2.0f * m_carDimensions.wheelBase_mm))));
    float dTheta = wheelDiff_mm / m_carDimensions.wheelBase_mm;

    m_odometry->dx = dx_mm;
    m_odometry->dy = dy_mm;
    m_odometry->dTheta = dTheta;

    m_odometry->globalX += dx_mm;
    m_odometry->globalY += dy_mm;
    m_odometry->globalTheta += dTheta;

}
void Car::main(){
    //drive(125, 0.0f);
}
