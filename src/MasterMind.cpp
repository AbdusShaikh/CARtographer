#include "MasterMind.h"
//TODO: Handle shutdown properly
MasterMind::MasterMind(){};
MasterMind::~MasterMind(){};

int MasterMind::init()
{
#if !DISABLE_LIDAR
    if (m_lidar.init(&m_observations.lidarExtractedLandmarks) == EXIT_FAILURE) {
        printf("[MasterMind]: Failed to initialize Lidar sensor. Exiting program\n");
        // exit(1);
        return EXIT_FAILURE;
    };
#endif

#if !DISABLE_CAR
    if (m_car.init(&m_observations.odometry) == EXIT_FAILURE) {
        printf("MasterMind]: Failed to initialize Car. Exiting program\n");
        // exit(1);
        return EXIT_FAILURE;
    };
#endif

#if !DISABLE_SLAM
    if(slamAlgo.init() == EXIT_FAILURE){
        printf("MasterMind]: Failed to initialize SLAM. Exiting program\n");
        return EXIT_FAILURE;
    }
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
        printf("[MasterMind]: Failed to uninitialize Car. Exiting program");
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

#if !DISABLE_CAR
        m_car.computeOdometry();
#endif

#if !DISABLE_SLAM
    slamAlgo.step(m_observations.lidarExtractedLandmarks, m_observations.odometry);
#endif

#if !DISABLE_CAR
        m_car.main();
#endif
    }
    return EXIT_SUCCESS;
}

int main(){
    MasterMind masterMind;
    if (masterMind.init() == EXIT_FAILURE){
        exit(EXIT_FAILURE);
    };
    masterMind.run();
    masterMind.uninit();
    return EXIT_SUCCESS;
};