#include "Lidar.h"

Lidar::Lidar(){

};

Lidar::~Lidar(){
    m_driver->stop();
	sleep(1);
    m_driver->setMotorSpeed(0);
    // done!
    if(m_driver) {
        delete m_driver;
        m_driver = NULL;
    }
}

int Lidar::init(){
    m_serialPort = "/dev/ttyUSB0";
    m_baudRate = 115200;
    m_driver = *createLidarDriver();
    if (!m_driver){
        printf("[RPLIDAR]: Insufficent memory, exit\n");
        return 1;
    }
    m_serialChannel = *createSerialPortChannel(m_serialPort, m_baudRate);
    if (!SL_IS_OK((m_driver->connect(m_serialChannel)))){
        printf("[RPLIDAR]: Failed to connect. Exiting\n");
        return 1;
    }
    sl_lidar_response_device_health_t healthinfo;
    if (!SL_IS_OK(m_driver->getHealth(healthinfo))) {
        printf("[RPLIDAR]: Bad RpLidar health. Exiting\n");
        return 1;
    }

    printf("[RPIDAR]: Connection Successful\n");

    m_driver->setMotorSpeed();
    // start scan...
    m_driver->startScan(0,1);

    printf("[RPIDAR]: Scanning Started\n");

    return 0;
}

int Lidar::scan(){
    size_t node_count = 8192;
    sl_lidar_response_measurement_node_hq_t nodes[node_count];

    if (SL_IS_OK(m_driver->grabScanDataHq(nodes, node_count))){
        m_driver->ascendScanData(nodes, node_count);
        for (int i = 0; i < (int)node_count; i++){
            printf("Node: %d | Flag: %d | Angle: %d | Distance: %d | Quality: %d \n",
            i,
            nodes[i].flag,
            nodes[i].angle_z_q14,
            nodes[i].dist_mm_q2,
            nodes[i].quality );
        }
    }
}
