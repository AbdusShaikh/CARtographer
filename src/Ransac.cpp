#include "Ransac.h"

Ransac::Ransac(){};
Ransac::~Ransac(){};

// ------------------------
// Main Algorithm functions
// ------------------------

void Ransac::init(const vector<scanDot> lidarPoints){
    maxAttempts = 1000;  
    initialSampleCount = 10;
    maxDistToLine = 5; // mm
    minLinePointCount = 90;

    m_unassociatedPoints.clear();
    m_associatedPoints.clear();
    m_extractedLines.clear();
    m_unassociatedPoints = convertPointsToCartesian(lidarPoints);

}

void Ransac::run(){
    int currAttempts = 0;
    std::random_device rd;
    std::mt19937 gen(rd());
    while (currAttempts < maxAttempts && m_unassociatedPoints.size() >= minLinePointCount){
        std::uniform_int_distribution<int> dist(0, m_unassociatedPoints.size() - 1); // Get a random index into m_unassociatedPoints
        int startIdx = dist(gen); // Where to start construction of line of best fit
        int endIdx = startIdx + initialSampleCount; // Where to end construction of line of best fit
        if (endIdx > (int) m_unassociatedPoints.size() - 1){
            startIdx -= (endIdx - (m_unassociatedPoints.size() - 1)); // How far away is endIdx from max index value
            endIdx = m_unassociatedPoints.size() - 1;
        }
        startIdx = max(0, startIdx); // Cut startIdx off at 0

        Line fittedLine = fitLine(m_unassociatedPoints, startIdx, endIdx);
        testLine(fittedLine);
        currAttempts ++;
    }

#if DISPLAY_EXTRACTED_LINES_RANSAC
    displayExtractedLines();
#endif
}

// --------------------------
// Algorithm helper functions
// --------------------------

// Fit a line through "samplePoints" according to least squares method
Line Ransac::fitLine(const vector<Point2f> samplePoints, int startIdx, int endIdx){
    //Calculate sums of x and y points
    float sumX = 0, sumY = 0;
    float numPoints = endIdx - startIdx + 1;
    for (int i = startIdx; i <= endIdx; i++){
        sumX += samplePoints[i].x;
        sumY += samplePoints[i].y;
    }
    float meanX = sumX / numPoints;
    float meanY = sumY / numPoints;
    
    // Calculate slope of line of best fit
    float slopeNumerator = 0, slopeDenominator = 0;
    for (int i = startIdx; i <= endIdx; i++){
        float xDiff = (samplePoints[i].x - meanX);
        float yDiff = (samplePoints[i].y - meanY);
        slopeNumerator += xDiff * yDiff;
        slopeDenominator += xDiff * xDiff;
    }
    // Define line of best fit
    float slope = slopeNumerator / slopeDenominator;
    float yIntercept = meanY - (slope * meanX);
    
    // y = mx + b
    // 0 = mx -y + b
    // Convert to line of the form ax + by + c = 0
    float a = slope;
    float b = -1;
    float c = yIntercept; 

    Line fittedLine = {a, b, c};
    return fittedLine;
}

// Iterate through each point in the point cloud and determine if it is within maxDistToLine from the line given by ax + by + c = 0
//  - If yes: 
//      - Add this point to local associatedPoints vector and increase the point count of the line.
//  - If no: 
//      - Add this point to local unassociatedPoints vector
// After iterating through all points
//  - Check if minLinePointCount is met
//      - If yes: 
//          - This is a good line. Update global associatedPoints and unassociatedPoints.
//          - Fit a new line through these points and add to m_extractedLine.
//      - If no: This is a bad line. Do not update anything and move on
void Ransac::testLine(Line line){
    vector<Point2f> localAssociatedPoints;
    vector<Point2f> localUnassociatedPoints;
    
    for (int i = 0; i < (int) m_unassociatedPoints.size(); i ++){
        if (distPointToLine(m_unassociatedPoints[i], line) <= maxDistToLine){
            localAssociatedPoints.push_back(m_unassociatedPoints[i]);
        }
        else {
            localUnassociatedPoints.push_back(m_unassociatedPoints[i]);
        }
    }
    
    if (localAssociatedPoints.size() >= minLinePointCount){
        m_unassociatedPoints = localUnassociatedPoints;
        m_associatedPoints.insert(m_associatedPoints.end(), localAssociatedPoints.begin(), localAssociatedPoints.end()); 
        Line extractedLine = fitLine(localAssociatedPoints, 0, localAssociatedPoints.size() - 1);
        m_extractedLines.push_back(extractedLine);
    }
}

// -----------------
// Utility functions
// -----------------

// Distance from a point to a line in cartesian coordinates
float Ransac::distPointToLine(Point2f point, Line line){
    float numerator = fabs((line.a * point.x) + (line.b * point.y) + line.c);
    float denominator = sqrt((line.a * line.a) + (line.b * line.b));
    float dist = numerator / denominator;

    return dist;
}

// Convert all given lidarPoints from polar coordinates to cartesian coordinates
vector<Point2f> Ransac::convertPointsToCartesian(const vector<scanDot> lidarPoints){
    vector<Point2f> cartesianPoints;
    for (int i = 0; i <  (int) lidarPoints.size(); i++){
        float x = lidarPoints[i].dist * cos(lidarPoints[i].angle);
        float y = lidarPoints[i].dist * sin(lidarPoints[i].angle);
        cartesianPoints.push_back(Point2f(x, y));
    }
    return cartesianPoints;
};

#if DISPLAY_EXTRACTED_LINES_RANSAC
void Ransac::displayExtractedLines(){
    Mat image = Mat::zeros(800, 800, CV_8UC3);
    Point center = Point(image.rows / 2, image.cols / 2);
    circle(image, center, 5, Scalar(0, 255,0));
    int scaleFactor = 20;
    for (int i = 0; i < (int) m_extractedLines.size(); i++){
        // ax + by + c = 0
        // y = -(ax + b) / b
        Line currLine = m_extractedLines[i];
        float x1 = image.cols;
        float y1 = -((currLine.a * x1) + (currLine.c / scaleFactor)) / currLine.b;
        float x2 = -image.cols;
        float y2 = -((currLine.a * x2) + (currLine.c / scaleFactor)) / currLine.b;

        Point displayPoint1 = Point(center.x + x1, center.y - (y1));
        Point displayPoint2 = Point(center.x + x2, center.y - (y2));
        Scalar pointColour = Scalar(0, 255, 0);
        line(image, displayPoint1, displayPoint2, pointColour);
    }
    imshow("RANSAC", image);
    waitKey(1);
    return;
}
#endif