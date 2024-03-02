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
        int init();
        int scan();
        void displayLidarData();
    private:
        string m_serialPort = "/dev/ttyUSB0";
        int m_baudRate = 115200;
        ILidarDriver * m_driver;
        IChannel* m_serialChannel;
        size_t m_nodeCount = 8192;
        sl_lidar_response_measurement_node_hq_t m_nodes[8192];
};