#include "MasterMind.h"

MasterMind::MasterMind(){};
MasterMind::~MasterMind(){};

int MasterMind::init()
{
#if !DISABLE_LIDAR
    if (m_lidar.init(&m_observations.lidarReadings) == EXIT_FAILURE) {
        printf("[MasterMind]: Failed to initialize Lidar sensor. Exiting program\n");
        // exit(1);
        return EXIT_FAILURE;
    };
#endif

#if !DISABLE_CAR
    if (m_car.init(m_observations.wheelEncoderReadings) == EXIT_FAILURE) {
        printf("MasterMind]: Failed to initialize Car. Exiting program\n");
        // exit(1);
        return EXIT_FAILURE;
    };
#endif
    return EXIT_SUCCESS;
}

int MasterMind::uninit(){
#if !DISABLE_LIDAR
    if (m_lidar.uninit() == EXIT_FAILURE){
        printf("[MasterMind]: Failed to uninitialize Lidar sensor. Exiting program");
        // exit(1);
        return EXIT_FAILURE;
    }
#endif
#if !DISABLE_CAR
    if (m_car.uninit() == EXIT_FAILURE){
        printf("[MasterMind]: Failed to uninitialize Lidar sensor. Exiting program");
        // exit(1);
        return EXIT_FAILURE;
    }
#endif
    return EXIT_SUCCESS;
}

int MasterMind::run(){
    while (true){
#if !DISABLE_LIDAR
        m_lidar.main();
#endif
    assert(m_observations.lidarReadings.size() > 0);
    float distThreshold_mm = 100.0f;
    float angleThreshold = 0.05f;
    float minLineLength = 30;
    vector<vector<scanDot>> extractedLines = m_LineExtractor.splitAndMerge(m_observations.lidarReadings, distThreshold_mm, angleThreshold, minLineLength);
    

#if !DISABLE_CAR
        m_car.main();
#endif
    }
    return EXIT_SUCCESS;
}

int main(){
    MasterMind masterMind;
    masterMind.init();
    masterMind.run();
    return EXIT_SUCCESS;
};