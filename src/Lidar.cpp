#include "Lidar.h"

Lidar::Lidar(){};

Lidar::~Lidar(){}

int Lidar::init(vector<vector<scanDot>> *lidarFeatureDeposit ){
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
    m_lineFeatures = lidarFeatureDeposit;
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

    m_nodes.clear();
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
            m_nodes.push_back(dot);
        }
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}

void Lidar::displayLidarData(){
    Mat image = Mat::zeros(800, 800, CV_8UC3);
    Point center = Point(400, 400);
    circle(image, center, 5, Scalar(0, 255,0));
    int scaleFactor = 20;
    for (int i = 0; i < (int) m_nodes.size(); i++){
        scanDot currNode = m_nodes[i];
        if (!currNode.dist) continue;
        Point cartesianPoint = Point(center.x + ((currNode.dist / scaleFactor) * cos(currNode.angle)), center.y - ((currNode.dist / scaleFactor) * sin(currNode.angle)));
        Scalar pointColour = Scalar(255, 0, 0);
        circle(image, cartesianPoint, 3, pointColour, 2);
    }
    imshow("Lidar Data", image);
    waitKey(1);
    return;
}

void Lidar::main(){
    if (scan() == EXIT_FAILURE){
        printf("[RPLIDAR]: Failure to scan in main loop. Exiting\n");
        return;
    }
    assert(m_nodes.size() > 0);
#if DISPLAY_LIDAR_READINGS
    displayLidarData();
#endif
#if USE_SPLITANDMERGE
    float distThreshold_mm = 100.0f;
    float angleThreshold = 0.05f;
    float minLineLength = 10;
    vector<vector<scanDot>> extractedLines = m_LineExtractor.splitAndMerge(m_nodes, distThreshold_mm, angleThreshold, minLineLength);
    *m_lineFeatures = extractedLines;
#endif 

#if USE_RANSAC
    m_lineExtractorRansac.init(m_nodes);
    m_lineExtractorRansac.run();
#endif
    return;
}