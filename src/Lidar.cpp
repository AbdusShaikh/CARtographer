#include "Lidar.h"

Lidar::Lidar(){};

Lidar::~Lidar(){}

int Lidar::init(vector<scanDot>* lidarReadingCollector ){
    m_driver = *createLidarDriver();
    if (!m_driver){
        printf("[RPLIDAR]: Insufficent memory, exit\n");
        return EXIT_FAILURE;
    }
    m_serialChannel = *createSerialPortChannel(m_serialPort, m_baudRate);
    if (!SL_IS_OK((m_driver->connect(m_serialChannel)))){
        printf("[RPLIDAR]: Failed to connect. Exiting\n");
        return EXIT_FAILURE;
    }
    sl_lidar_response_device_health_t healthinfo;
    if (!SL_IS_OK(m_driver->getHealth(healthinfo))) {
        printf("[RPLIDAR]: Bad RpLidar health. Exiting\n");
        return EXIT_FAILURE;
    }

    printf("[RPLIDAR]: Connection Successful\n");
    m_nodes = lidarReadingCollector;
    m_driver->setMotorSpeed();
    // start scan...
    m_driver->startScan(0,1);
    printf("[RPLIDAR]: Scanning Started\n");
    // namedWindow("Lidar Data", WINDOW_AUTOSIZE);
    return EXIT_SUCCESS;
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
    return EXIT_SUCCESS;
}

int Lidar::scan(){

    (*m_nodes).clear();
    size_t nodeCount = 8192;
    sl_lidar_response_measurement_node_hq_t newNodes[nodeCount];

    if (SL_IS_OK(m_driver->grabScanDataHq(newNodes, nodeCount))){
        m_driver->ascendScanData(newNodes, nodeCount);
        for (int i = 0; i < (int)nodeCount; i++){
            //if (!newNodes[i].dist_mm_q2) continue;
            scanDot dot;
            dot.dist = newNodes[i].dist_mm_q2;
            float angle_deg = (newNodes[i].angle_z_q14 *90.0f) / 16384.0f;
            dot.angle = (angle_deg * M_PI) / 180.0f;
            dot.quality = newNodes[i].quality;
            (*m_nodes).push_back(dot);
        }
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}

void Lidar::displayLidarData(){
    Mat image = Mat::zeros(800, 800, CV_8UC3);
    Point center = Point(400, 400);
    float c = 0;
    float s = 0;
    circle(image, center, 5, Scalar(0, 255,0));
    for (int i = 0; i < (int) (*m_nodes).size(); i++){
        scanDot currNode = (*m_nodes)[i];
        if (!currNode.dist) continue;
        int pointDist = currNode.dist / 10;
        Point det = Point(400, 400 - pointDist);
        float angle = currNode.angle;
        printf("[RPLIDAR]: Node[%d] Distance[%d] | Angle[%f]\n", i, pointDist, angle);
        c = cos(angle);
        s = sin(angle);
        float dx = det.x - center.x;
        float dy = det.y - center.y;
        Point rotatedDet = Point((dx * c) - (dy * s) + center.x,  (dx * s) + (dy * c) + center.y);

        rotatedDet = Point(center.x + (currNode.dist / 10) * c, center.y - (currNode.dist / 10) * s);

        Scalar pointColour = Scalar(255, 0, 0);
        // if (m_nodes[i].angle <= 90){
        //     pointColour = Scalar(0, 0, 255);
        // }
        // else if (m_nodes[i].angle >= 270){
        //     pointColour = Scalar(0, 255, 0);
        // }
        circle(image, rotatedDet, 3, pointColour, 2);
    }
    // float anteriorDist = getAvgAnteriorProximity();
    // Scalar movementTextColor = Scalar(0, 255, 0);
    // string movementText = "GO";
    // if (anteriorDist <= 2000){
    //     movementTextColor = Scalar(0,0,255);
    //     movementText = "STOP";
    // }
    // else if (anteriorDist <= 4000) {
    //     movementTextColor = Scalar(0, 255, 255);
    //     movementText = "SLOW";
    // }
    // putText(image, movementText, center, FONT_HERSHEY_SIMPLEX, 4, movementTextColor, 3);
    imshow("Lidar Data", image);
    waitKey(1);
    return;
}

void Lidar::main(){
    if (scan() == EXIT_FAILURE){
        printf("[RPLIDAR]: Failure to scan in main loop. Exiting\n");
        return;
    }
    displayLidarData();
    return;
}