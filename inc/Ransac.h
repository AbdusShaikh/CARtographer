#include "common.h"
#include <random>
#include <opencv2/core/types.hpp>
#define DISPLAY_EXTRACTED_LINES_RANSAC 0
#define DEBUG_RANSAC 0

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
        vector<scanDot> run();
    private:
        // Algorithm hlper functions
        Line fitLine(const vector<Point2f> samplePoints, int startIdx, int endIdx);
        void testLine(const Line line);
        void extractLandmarks();
        void mergeLandmarks();
        // Utility Functions
        vector<Point2f> convertPointsToCartesian(const vector<scanDot> lidarPoints);
        float distPointToLine(const Point2f point, const Line line);
        float distPointToPoint(const Point2f p1, const Point2f p2);
#if DISPLAY_EXTRACTED_LINES_RANSAC
        void displayExtractedLines();
#endif

        vector<Point2f> m_lidarPointsCartesian;
        vector<Point2f> m_associatedPoints;
        vector<Point2f> m_unassociatedPoints;
        vector<Line> m_extractedLines;
        vector<scanDot> m_extractedLandmarks;

        int maxAttempts;
        int initialSampleCount;
        int degreesFromInitialSample;
        float maxDistToLine;
        float minLinePointCount;
        float landmarkMergeThreshold;

};