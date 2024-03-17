#include "Lidar.h"

Lidar::Lidar(){

};

Lidar::~Lidar(){}

int Lidar::init(iFace* interface){
    m_driver = *createLidarDriver();
    if (!m_driver){
        printf("[RPLIDAR]: Insufficent memory, exit\n");
        exit(1);
    }
    m_serialChannel = *createSerialPortChannel(m_serialPort, m_baudRate);
    if (!SL_IS_OK((m_driver->connect(m_serialChannel)))){
        printf("[RPLIDAR]: Failed to connect. Exiting\n");
        exit(1);

    }
    sl_lidar_response_device_health_t healthinfo;
    if (!SL_IS_OK(m_driver->getHealth(healthinfo))) {
        printf("[RPLIDAR]: Bad RpLidar health. Exiting\n");
        exit(1);
    }

    printf("[RPLIDAR]: Connection Successful\n");

    m_driver->setMotorSpeed();
    // start scan...
    m_driver->startScan(0,1);
    m_interface = interface;
    printf("[RPLIDAR]: Scanning Started\n");
    // namedWindow("Lidar Data", WINDOW_AUTOSIZE);
    return 0;
}

int Lidar::uninit(){
    m_driver->stop();
	sleep(1);
    m_driver->setMotorSpeed(0);
    // done!
    if(m_driver) {
        delete m_driver;
        m_driver = NULL;
    }
    return 1;
}

int Lidar::scan(){
    m_nodes.clear();
    size_t nodeCount = 8192;
    sl_lidar_response_measurement_node_hq_t newNodes[nodeCount];

    if (SL_IS_OK(m_driver->grabScanDataHq(newNodes, nodeCount))){
        m_driver->ascendScanData(newNodes, nodeCount);
        for (int i = 0; i < (int)nodeCount; i++){
            //if (!newNodes[i].dist_mm_q2) continue;
            scanDot dot;
            dot.dist = newNodes[i].dist_mm_q2;
            dot.angle = (newNodes[i].angle_z_q14 *90.0f) / 16384.0f;
            dot.quality = newNodes[i].quality;
            m_nodes.push_back(dot);
        }
        return 0;
    }
    return 1;
}

float Lidar::getAvgAnteriorProximity(){
    float totalDist = 0;
    float count = 0;

    for (int i = 0; i < (int) m_nodes.size(); i++){
        // Only count front of car
        if (!m_nodes[i].dist || (m_nodes[i].angle > 45.0f && m_nodes[i].angle < 315.0f)) continue;
        totalDist += m_nodes[i].dist;
        count += 1;
    }

    return totalDist / count;
}

void Lidar::displayLidarData(){
    Mat image = Mat::zeros(800, 800, CV_8UC3);
    Point center = Point(400, 400);
    float c = 0;
    float s = 0;
    circle(image, center, 5, Scalar(0, 255,0));
    for (int i = 0; i < (int) m_nodes.size(); i++){
        if (!m_nodes[i].dist) continue;
        int pointDist = m_nodes[i].dist / 10;
        Point det = Point(400, 400 - pointDist);
        float detAngleRad = (m_nodes[i].angle * M_PI) / 180.0f;
        printf("[RPLIDAR]: Node[%d] Distance[%d] | Angle[%f]\n", i, pointDist, m_nodes[i].angle);
        c = cos(detAngleRad);
        s = sin(detAngleRad);
        float dx = det.x - center.x;
        float dy = det.y - center.y;
        Point rotatedDet = Point((dx * c) - (dy * s) + center.x,  (dx * s) + (dy * c) + center.y);
        Scalar pointColour = Scalar(255, 0, 0);
        // if (m_nodes[i].angle <= 90){
        //     pointColour = Scalar(0, 0, 255);
        // }
        // else if (m_nodes[i].angle >= 270){
        //     pointColour = Scalar(0, 255, 0);
        // }
        circle(image, rotatedDet, 3, pointColour, 2);
    }
    float anteriorDist = getAvgAnteriorProximity();
    Scalar movementTextColor = Scalar(0, 255, 0);
    string movementText = "GO";
    if (anteriorDist <= 2000){
        movementTextColor = Scalar(0,0,255);
        movementText = "STOP";
    }
    else if (anteriorDist <= 4000) {
        movementTextColor = Scalar(0, 255, 255);
        movementText = "SLOW";
    }
    putText(image, movementText, center, FONT_HERSHEY_SIMPLEX, 4, movementTextColor, 3);
    imshow("Lidar Data", image);
    waitKey(1);
    return;
}

void Lidar::main(){
    if (scan()){
        printf("[RPLIDAR]: Failure to scan in main loop. Exiting\n");
        return;
    }
    float anteriorDist = getAvgAnteriorProximity();
    m_interface->isClear = true;
    if (anteriorDist <= 2000){
        m_interface->isClear = false;
    }
    return;
}