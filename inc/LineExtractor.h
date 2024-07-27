#include "common.h"

using namespace std;

struct PolarLine{
    float r;
    float theta;
};

struct CartesianLine {
    float A;
    float B;
    float C;
};

class LineExtractor{
    public:
        void splitAndMerge(vector<scanDot> lidarPoints, float distThreshold);
    private:
        vector<scanDot> split(vector<scanDot> lidarPoints, float distThreshold);
        vector<scanDot> merge(vector<scanDot> segments, float distThreshold);
        void fitLine(scanDot point1, scanDot point2, CartesianLine* resultLine);
        float perpDist(scanDot point, CartesianLine line);
};