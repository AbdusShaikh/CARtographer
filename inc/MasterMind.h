#include "Car.h"
#include "Lidar.h"
#include "LineExtractor.h"
#include "common.h"

#define DISABLE_LIDAR 1
#define DISABLE_CAR 0

class MasterMind {
    public:
        MasterMind();
        ~MasterMind();
        int init();
        int uninit();
        int run();
    private:
        int updateMap();
        int pathPlan();
        LineExtractor m_LineExtractor;
        Observation m_observations;
        Lidar m_lidar;
        Car m_car;
};