#include "common.h"
#include "sl_lidar_driver.h"
#include "sl_lidar.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using  namespace std;
using  namespace sl;
using namespace cv;

class Lidar{
    public:
        Lidar();
        ~Lidar();
        int init(vector<scanDot>* lidarReadingCollector);
        int uninit();
        int scan();
        void displayLidarData();
        void main();

    private:
        string m_serialPort = "/dev/ttyUSB0";
        int m_baudRate = 115200;
        ILidarDriver * m_driver;
        IChannel* m_serialChannel;
        vector<scanDot> *m_nodes;
};