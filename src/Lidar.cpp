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
    namedWindow("Lidar Data", WINDOW_AUTOSIZE);
    return 0;
}

int Lidar::scan(){

    if (SL_IS_OK(m_driver->grabScanDataHq(m_nodes, m_nodeCount))){
        m_driver->ascendScanData(m_nodes, m_nodeCount);
        // for (int i = 0; i < (int)m_nodeCount; i++){
        //     printf("Node: %d | Flag: %d | Angle: %d | Distance: %d | Quality: %d \n",
        //     i,
        //     m_nodes[i].flag,
        //     m_nodes[i].angle_z_q14,
        //     m_nodes[i].dist_mm_q2,
        //     m_nodes[i].quality );
        // }
        return 0;
    }
    return 1;
}

void Lidar::displayLidarData(){
    Mat image = Mat::zeros(800, 800, CV_8UC3);
    Point center = Point(400, 400);
    float32_t c = 0;
    float32_t s = 0;
    circle(image, center, 5, Scalar(0, 255,0));
    for (int i = 0; i < (int) m_nodeCount; i++){
        int point_dist = m_nodes[i].dist_mm_q2 / 10;
        if (point_dist == 0) continue;
        printf("[RPLIDAR]: Node[%d] Distance[%d] | Angle[%f]\n", i, point_dist, (m_nodes[i].angle_z_q14 * 90.f) / 16384.0f);
        Point det = Point(400, 400 - point_dist);
        float32_t det_angle = (m_nodes[i].angle_z_q14 * 90.0f) / 16384.0f;
        float32_t det_angle_rad = (det_angle * 3.14150) / 180.0f;
        c = cos(det_angle_rad);
        s = sin(det_angle_rad);
        float32_t dx = det.x - center.x;
        float32_t dy = det.y - center.y;
        Point rotatedDet = Point((dx * c) - (dy * s) + center.x,  (dx * s) + (dy * c) + center.y); 
        circle(image, rotatedDet, 3, Scalar(255, 0,0), 2);
    }
    imshow("Lidar Data", image);
    waitKey(1);
    return;
}
