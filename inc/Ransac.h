#include "common.h"
#include <random>
#include <opencv2/core/types.hpp>
#define DISPLAY_EXTRACTED_LINES_RANSAC 1

#if DISPLAY_EXTRACTED_LINES_RANSAC
    #include <opencv2/opencv.hpp>
    #include <opencv2/highgui.hpp>
    #include <opencv2/imgproc.hpp>
#endif

using namespace cv;

struct Line{
    float a;
    float b;
    float c;
};

class Ransac{
    public:
        Ransac();
        ~Ransac();
        void init(const vector<scanDot> lidarPoints);
        void run();
    private:
        // Algorithm functions
        Line fitLine(const vector<Point2f> samplePoints, int startIdx, int endIdx);
        void testLine(Line line);
        // Utility Functions
        vector<Point2f> convertPointsToCartesian(const vector<scanDot> lidarPoints);
        float distPointToLine(Point2f point, Line line);
#if DISPLAY_EXTRACTED_LINES_RANSAC
        void displayExtractedLines();
#endif

        vector<Point2f> m_lidarPointsCartesian;
        vector<Point2f> m_associatedPoints;
        vector<Point2f> m_unassociatedPoints;
        vector<Line> m_extractedLines;

        int maxAttempts;
        int initialSampleCount;
        int degreesFromInitialSample;
        float maxDistToLine;
        float minLinePointCount;
};