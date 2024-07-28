#include "common.h"

using namespace std;

struct CartesianCoord {
    float x;
    float y;
};

class LineExtractor{
    public:
        vector<vector<scanDot>>  splitAndMerge(vector<scanDot> lidarPoints, float distThreshold, float angThreshold);
    private:
        CartesianCoord polarToCartesian(scanDot polarPoint);
        CartesianCoord normalizeVector(CartesianCoord vector);
        CartesianCoord cartesianPointsToVector(CartesianCoord vector1, CartesianCoord vector2);
        float dotProduct(CartesianCoord vector1, CartesianCoord vector2);

        vector<vector<scanDot>> split(vector<scanDot> lidarPoints, float distThreshold);
        vector<vector<scanDot>> merge(vector<vector<scanDot>> splitSegments, float angThreshold);
        float pointToLineDist(scanDot linePoint1, scanDot linePoint2, scanDot testPoint);
};