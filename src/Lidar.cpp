#include "Lidar.h"

Lidar::Lidar(){};

Lidar::~Lidar(){}

int Lidar::init(vector<scanDot> *lidarFeatureDeposit ){
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
    // m_lineFeatures = lidarFeatureDeposit;
    m_lidarFeatureDeposit = lidarFeatureDeposit;
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
    
    // Grab scan data from Lidar (Non-blocking operation)
    if (SL_IS_OK(m_driver->grabScanDataHq(newNodes, nodeCount, 0))){
        for (int i = 0; i < (int)nodeCount; i++){
            if (!newNodes[i].dist_mm_q2 || newNodes[i].dist_mm_q2 >= 3500) continue;

            scanDot dot;
            dot.dist = newNodes[i].dist_mm_q2 / 4.0f;
            float angle_deg = (newNodes[i].angle_z_q14 *90.0f) / 16384.0f;
            // Adapted for position of Lidar on Car
            dot.angle = -(angle_deg * M_PI) / 180.0f;
            m_nodes.push_back(dot);
        }
        std::sort(m_nodes.begin(), m_nodes.end(), [](const scanDot& a, const scanDot& b) {return a.angle < b.angle;});
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}

#if DISPLAY_LIDAR_READINGS
void Lidar::displayLidarData(){
    Mat image = Mat::zeros(800, 800, CV_8UC3);
    image.setTo(cv::Scalar(DISPLAY_BACKGROUND_COLOUR)); // Make the image grey
    Point center = Point(400, 400);
    circle(image, center, 5, Scalar(0, 255,0), cv::FILLED);
    // int scaleFactor = 20;
    for (int i = 0; i < (int) m_nodes.size(); i++){
        scanDot currNode = m_nodes[i];
        if (!currNode.dist) continue;
        Point cartesianPoint = Point(center.x + ((currNode.dist / DISPLAY_SCALE) * cos(currNode.angle)), center.y - ((currNode.dist / DISPLAY_SCALE) * sin(currNode.angle)));
        Scalar pointColour = Scalar(255, 0, 0);
        circle(image, cartesianPoint, 3, pointColour, 2);
    }
    imshow("Lidar Data", image);
    waitKey(1);
    return;
}
#endif 

#if DUMP_LIDAR_READINGS
void Lidar::dumpLidarReadings(){
    std::ofstream outFile("LidarDump.txt", std::ios::app);

    if (!outFile) {
        std::cerr << "Error opening file for writing." << std::endl;
    }
    
    float rangeSum = 0, angleSum = 0;
    for (int i = 0; i < (int) m_nodes.size(); i++){
        rangeSum += m_nodes[i].dist;
        angleSum += m_nodes[i].angle;
    }
    float rangeMean = rangeSum / m_nodes.size();
    float angleMean = angleSum / m_nodes.size();
    outFile << rangeMean << ", " << angleMean << ", " << m_nodes.size() << std::endl;

    // Close the file
    outFile.close();
}
#endif

int Lidar::main(){
    int scanResult = scan();
    if (scanResult == EXIT_SUCCESS){
    assert(m_nodes.size() > 0);
    #if DISPLAY_LIDAR_READINGS
        displayLidarData();
    #endif
    #if DUMP_LIDAR_READINGS
        dumpLidarReadings();
    #endif
        m_lineExtractorRansac.init(m_nodes);
        vector<scanDot> extractedLandmarks = m_lineExtractorRansac.run();
        *m_lidarFeatureDeposit = extractedLandmarks;
    }
    return scanResult;
}