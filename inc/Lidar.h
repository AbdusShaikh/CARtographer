#include "common.h"
#include "sl_lidar_driver.h"
#include "sl_lidar.h"
#include "Ransac.h"


#define DISPLAY_LIDAR_READINGS 0
#define DUMP_LIDAR_READINGS 0


#if DISPLAY_LIDAR_READINGS
    #include <opencv2/opencv.hpp>
    #include <opencv2/highgui.hpp>
    #include <opencv2/imgproc.hpp>
#endif

#if DUMP_LIDAR_READINGS
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
        int main();
        int uninit();

    private:
        int scan();
        void displayLidarData();
#if DUMP_LIDAR_READINGS
        void dumpLidarReadings();
#endif
        Ransac m_lineExtractorRansac;
        string m_serialPort = "/dev/ttyUSB0";
        int m_baudRate = 115200;
        ILidarDriver * m_driver;
        IChannel* m_serialChannel;
        vector<scanDot> m_nodes;
        vector<scanDot> *m_lidarFeatureDeposit;
};