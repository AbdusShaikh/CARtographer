#include "Car.h"
#include "Lidar.h"
#include "common.h"

class MasterMind {
    public:
        MasterMind();
        ~MasterMind();
        int init();
        int uninit();
        int run();
    private:
        int lidarFeatureExtract();
        int updateMap();
        int pathPlan();
        Observation m_observations;
        Lidar m_lidar;
        Car m_car;

};