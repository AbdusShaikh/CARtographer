#include "MasterMind.h"
volatile int ctrlCFlag = 0;

void sigintHandler(int sig_num){
    ctrlCFlag = 1;
    printf("ctrl-c pressed\n");
}

MasterMind::MasterMind(){};
MasterMind::~MasterMind(){};

int MasterMind::init()
{
    if (m_lidar.init(m_observations.lidarReadings) == EXIT_FAILURE) {
        printf("[MasterMind]: Failed to initialize Lidar sensor. Exiting program\n");
        // exit(1);
        return EXIT_FAILURE;
    };
    if (m_car.init(m_observations.wheelEncoderReadings) == EXIT_FAILURE) {
        printf("MasterMind]: Failed to initialize Car. Exiting program\n");
        // exit(1);
        return EXIT_FAILURE;
    };
    
    return EXIT_SUCCESS;
}

int MasterMind::uninit(){
    if (m_lidar.uninit() == EXIT_FAILURE){
        printf("[MasterMind]: Failed to uninitialize Lidar sensor. Exiting program");
        // exit(1);
        return EXIT_FAILURE;
    }
    if (m_car.uninit() == EXIT_FAILURE){
        printf("[MasterMind]: Failed to uninitialize Lidar sensor. Exiting program");
        // exit(1);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

int MasterMind::run(){
    m_lidar.main();
    m_car.main();
    return EXIT_SUCCESS;
}

int main(){
    signal(SIGINT, sigintHandler);
    MasterMind masterMind;
    masterMind.init();
    masterMind.run();
    return EXIT_SUCCESS;
};