#include "Car.h"
#include "Lidar.h"
#include "LineExtractor.h"
#include "common.h"

#define DISABLE_LIDAR 0
#define DISABLE_CAR 1

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
        
        LineExtractor m_LineExtractor;
        Observation m_observations;
        Lidar m_lidar;
        Car m_car;


};