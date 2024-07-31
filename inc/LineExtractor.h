#define DISPLAY_EXTRACTED_LINES 1

#include "common.h"
#if DISPLAY_EXTRACTED_LINES
    #include <opencv2/opencv.hpp>
    #include <opencv2/highgui.hpp>
    #include <opencv2/imgproc.hpp>
    using namespace cv;
#endif
using namespace std;

struct CartesianCoord {
    float x;
    float y;
};

class LineExtractor{
    public:
        vector<vector<scanDot>>  splitAndMerge(vector<scanDot> lidarPoints, float distThreshold, float angThreshold);
    private:
        // Utitlity functions
        CartesianCoord polarToCartesian(scanDot polarPoint);
        CartesianCoord normalizeVector(CartesianCoord vector);
        CartesianCoord cartesianPointsToVector(CartesianCoord vector1, CartesianCoord vector2);
        float dotProduct(CartesianCoord vector1, CartesianCoord vector2);
        float pointToLineDist(scanDot linePoint1, scanDot linePoint2, scanDot testPoint);
        void displayExtractedLines(vector<vector<scanDot>> mergedSegments);

        // Algorithm functions
        vector<vector<scanDot>> split(vector<scanDot> lidarPoints, float distThreshold);
        vector<vector<scanDot>> merge(vector<vector<scanDot>> splitSegments, float angThreshold);
};