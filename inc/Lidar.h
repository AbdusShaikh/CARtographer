#include "common.h"
#include "sl_lidar_driver.h"
#include "sl_lidar.h"

using  namespace std;
using  namespace sl;

class Lidar{
    public:
        Lidar();
        ~Lidar();
        int init();
        int scan();
    private:
        string m_serialPort;
        int m_baudRate;
        ILidarDriver * m_driver;
        IChannel* m_serialChannel;
};