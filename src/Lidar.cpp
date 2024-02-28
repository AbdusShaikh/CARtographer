#include "Lidar.h"

int basicTest(){
	ILidarDriver * drv = *createLidarDriver();
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }
    IChannel* serial_channel = *createSerialPortChannel(LIDAR_SERIAL_PORT, LIDAR_BAUD_RATE);
    if (!SL_IS_OK((drv->connect(serial_channel)))){
        printf("[RPLIDAR]: Failed to connect. Exiting\n");
        return 1;
    }
    sl_lidar_response_device_health_t healthinfo;
    if (!SL_IS_OK(drv->getHealth(healthinfo))) {
        printf("[RPLIDAR]: Bad RpLidar health. Exiting\n");
        return 1;
    }
    printf("[RPIDAR]: Connection Successful");

    drv->setMotorSpeed();
    // start scan...
    drv->startScan(0,1);

    size_t node_count = 8192;
    sl_lidar_response_measurement_node_hq_t nodes[node_count];

    if (SL_IS_OK(drv->grabScanDataHq(nodes, node_count))){
        drv->ascendScanData(nodes, node_count);
        for (int i = 0; i < (int)node_count; i++){
            printf("Node: %d | Flag: %d | Angle: %d | Distance: %d | Quality: %d \n",
            i,
            nodes[i].flag,
            nodes[i].angle_z_q14,
            nodes[i].dist_mm_q2,
            nodes[i].quality );
        }
    }

    drv->stop();
	sleep(1);
    drv->setMotorSpeed(0);
    // done!
    if(drv) {
        delete drv;
        drv = NULL;
    }
    return 0;
}