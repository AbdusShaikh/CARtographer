#include "Car.h"
#include "Lidar.h"
#include "common.h"

#define DISABLE_LIDAR 0
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
        Observation m_observations;
        Lidar m_lidar;
        Car m_car;
};