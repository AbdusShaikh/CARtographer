#include "common.h"
#include "sl_lidar_driver.h"
#include "sl_lidar.h"
#include "SplitAndMerge.h"
#include "Ransac.h"


#define DISPLAY_LIDAR_READINGS 1
#define USE_SPLITANDMERGE 0
#define USE_RANSAC 1
#define DUMP_LIDAR_READINGS 1


#if DISPLAY_LIDAR_READINGS
    #include <opencv2/opencv.hpp>
    #include <opencv2/highgui.hpp>
    #include <opencv2/imgproc.hpp>
#endif

#if DUMP_LIDAR_READINGS
    #include <iostream>
    #include <fstream>
#endif

using  namespace std;
using  namespace sl;
using namespace cv;

class Lidar{
    public:
        Lidar();
        ~Lidar();
        int init(vector<scanDot> *lidarFeatureDeposit);
        void main();
        int uninit();

    private:
        int scan();
        void displayLidarData();
#if DUMP_LIDAR_READINGS
        void dumpLidarReadings();
#endif
#if USE_SPLITANDMERGE
        SplitAndMerge m_LineExtractorSplitAndMerge;
#endif
#if USE_RANSAC
        Ransac m_lineExtractorRansac;
#endif
        string m_serialPort = "/dev/ttyUSB0";
        int m_baudRate = 115200;
        ILidarDriver * m_driver;
        IChannel* m_serialChannel;
        vector<scanDot> m_nodes;
        // vector<vector<scanDot>> *m_lineFeatures;
        vector<scanDot> *m_lidarFeatureDeposit;


};