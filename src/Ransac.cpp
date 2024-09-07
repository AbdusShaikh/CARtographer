#include "Ransac.h"

Ransac::Ransac(){};
Ransac::~Ransac(){};

// ------------------------
// Main Algorithm functions
// ------------------------

void Ransac::init(const vector<scanDot> lidarPoints){
    maxAttempts = 250;  
    initialSampleCount = 30;
    maxDistToLine = 7.0f; // mm
    minLinePointCount = 75.0f;
    landmarkMergeThreshold = 35.0f;


    m_unassociatedPoints.clear();
    m_associatedPoints.clear();
    m_extractedLines.clear();
    m_extractedLandmarks.clear();
    m_unassociatedPoints = convertPointsToCartesian(lidarPoints);

}

vector<scanDot> Ransac::run(){
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
    extractLandmarks();
#if DISPLAY_EXTRACTED_LINES_RANSAC
    displayExtractedLines();
#endif
#if DEBUG_RANSAC
    printf("[RANSAC:] Attempts: %d\n", currAttempts);
#endif
    return m_extractedLandmarks;
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
void Ransac::testLine(const Line line){
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

// Find the closest point on each line to the robot (origin). Return this as the landmark
void Ransac::extractLandmarks(){
    for (int i = 0; i < (int) m_extractedLines.size(); i ++){
        Line currLine = m_extractedLines[i];
        float denom = (currLine.a * currLine.a) + (currLine.b * currLine.b);
        float x = - (currLine.a * currLine.c) / denom;
        float y = - (currLine.b * currLine.c) / denom;
        float r = sqrt((x * x) +(y * y));
        float theta = atan2(y, x);

        scanDot newLandmark;
        newLandmark.dist = r;
        newLandmark.angle = theta;
        m_extractedLandmarks.push_back(newLandmark);
    }
    mergeLandmarks();
}

// Some lines are close enough together to be considered the same line. Merge the resulting landmarks based on distance rather than merging the lines
void Ransac::mergeLandmarks(){
    vector<vector<scanDot>> similarLandmarks;
    vector<scanDot> mergedLandmarks;
    // Group all landmarks that are within at most (maxDist * 2) mm of eachother (distance is only checked from first point)
    std::sort(m_extractedLandmarks.begin(), m_extractedLandmarks.end(), [](const scanDot& a, const scanDot& b) {return a.dist < b.dist;});
    int i = 0;
    while (i < (int) m_extractedLandmarks.size()){
        scanDot currPoint = m_extractedLandmarks[i];
        if (i == (int) m_extractedLandmarks.size() - 1){
            mergedLandmarks.push_back(currPoint);
        }
        else {
            scanDot nextPoint = m_extractedLandmarks[i + 1];
            float dist = sqrt((currPoint.dist * currPoint.dist) + (nextPoint.dist * nextPoint.dist) - (2.0f * currPoint.dist * nextPoint.dist * cos(currPoint.angle - nextPoint.angle)));
            if (dist > landmarkMergeThreshold){
                mergedLandmarks.push_back(currPoint);
            }
        }
        i++;
    }
#if DEBUG_RANSAC
    int landmarksBefore = m_extractedLandmarks.size();
    int landmarksAfter = mergedLandmarks.size();
    if (landmarksBefore > landmarksAfter){
        printf("[RANSAC:] Merged Landmarks; %d -> %d\n", landmarksBefore, landmarksAfter);
    }
#endif
    m_extractedLandmarks = mergedLandmarks;
}

// -----------------
// Utility functions
// -----------------

// Distance from a point to a line in cartesian coordinates
float Ransac::distPointToLine(const Point2f point, const Line line){
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
    image.setTo(cv::Scalar(DISPLAY_BACKGROUND_COLOUR)); // Make the image grey
    Point center = Point(image.rows / 2, image.cols / 2);
    Scalar green = Scalar(0, 171, 20);
    Scalar red = Scalar(0,0,255);
    circle(image, center, 5, green, cv::FILLED);

    // int scaleFactor = 20;
    for (int i = 0; i < (int) m_extractedLines.size(); i++){
        // --- Display line ---
        // ax + by + c = 0
        // y = -(ax + b) / b
        Line currLine = m_extractedLines[i];
        float x1 = image.cols;
        float y1 = -((currLine.a * x1) + (currLine.c / DISPLAY_SCALE)) / currLine.b;
        float x2 = -image.cols;
        float y2 = -((currLine.a * x2) + (currLine.c / DISPLAY_SCALE)) / currLine.b;

        Point displayPoint1 = Point(center.x + x1, center.y - y1);
        Point displayPoint2 = Point(center.x + x2, center.y - y2);
        line(image, displayPoint1, displayPoint2, green);
    }

    for (int i = 0; i < (int) m_extractedLandmarks.size(); i++){
        scanDot currLandmark = m_extractedLandmarks[i];
        float x = currLandmark.dist * cos(currLandmark.angle);
        float y = currLandmark.dist * sin(currLandmark.angle);

        Point displayPoint = Point(center.x + (x / DISPLAY_SCALE), center.y - (y / DISPLAY_SCALE));
        circle(image, displayPoint, 3, red, cv::FILLED);
    }
    imshow("RANSAC", image);
    waitKey(1);
    return;
}
#endif