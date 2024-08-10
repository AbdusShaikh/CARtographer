#include "Ransac.h"

Ransac::Ransac(){};
Ransac::~Ransac(){};

void Ransac::init(vector<scanDot> lidarPoints){
    // int redundancyFactor = 7;
    // int lineCountHypothesis = 4;
    // maxAttempts = lineCountHypothesis * redundancyFactor;  
    // initialSampleCount = lidarPoints.size() / (3 * lineCountHypothesis);
    // // degreesFromInitialSample;
    // maxDistToLine = 300; // mm
    // minLinePointCount = (lidarPoints.size() / (lineCountHypothesis * redundancyFactor));

    maxAttempts = 1000;  
    initialSampleCount = 6;
    maxDistToLine = 300; // mm
    minLinePointCount = 10;

    m_unassociatedPoints.clear();
    m_associatedPoints.clear();
    m_extractedLines.clear();
    m_unassociatedPoints = convertPointsToCartesian(lidarPoints);

}

void Ransac::run(){
    int currAttempts = 0;
    std::default_random_engine generator;
    while (currAttempts < maxAttempts && m_unassociatedPoints.size() >= minLinePointCount){
        std::uniform_int_distribution<int> distribution(0, m_unassociatedPoints.size() - 1);
        //TODO: Always returns 0 for some reason
        int randIdx = distribution(generator);  // generate a random index into unassociatedPairs;
        int startIdx = randIdx - (initialSampleCount / 2); // Where to begin construction of line of best fit
        int endIdx = randIdx + (initialSampleCount / 2); // Where to end construction of line of best fit
        if (startIdx < 0){ // If startIdx < 0, add the residual to endIdx (to maintain line length) and set startIdx to 0
            endIdx += abs(startIdx);
            startIdx = 0;
        }
        endIdx = min(endIdx, (int) m_unassociatedPoints.size() - 1);

        Line fittedLine = fitLine(m_unassociatedPoints, startIdx, endIdx);
        testLine(fittedLine);
        currAttempts ++;
    }

#if DISPLAY_EXTRACTED_LINES_RANSAC
    displayExtractedLines();
#endif
}

// Fit a line through "samplePoints" according to least squares
Line Ransac::fitLine(vector<cv::Point> samplePoints, int startIdx, int endIdx){
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
    vector<cv::Point> localAssociatedPoints;
    vector<cv::Point> localUnassociatedPoints;
    
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
        m_associatedPoints = localAssociatedPoints;
        Line extractedLine = fitLine(localAssociatedPoints, 0, localAssociatedPoints.size() - 1);
        m_extractedLines.push_back(extractedLine);
    }
}

// Distance from a point to a line in cartesian coordinates
float Ransac::distPointToLine(cv::Point point, Line line){
    float numerator = fabs((line.a * point.x) + (line.b * point.y) + line.c);
    float denominator = sqrt((line.a * line.a) + (line.b * line.b));
    float dist = numerator / denominator;

    return dist;
}

// Convert all given lidarPoints from polar coordinates to cartesian coordinates
vector<cv::Point> Ransac::convertPointsToCartesian(vector<scanDot> lidarPoints){
    vector<cv::Point> cartesianPoints;
    for (int i = 0; i <  (int) lidarPoints.size(); i++){
        float x = lidarPoints[i].dist * cos(lidarPoints[i].angle);
        float y = lidarPoints[i].dist * sin(lidarPoints[i].angle);
        cartesianPoints.push_back(cv::Point(x, y));
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
        // by = -ax - c
        // y = (-ax - c) / b
        Line currLine = m_extractedLines[i];
        float x1 = image.cols + 100;
        float y1 = ((-currLine.a * x1) - currLine.c) / currLine.b;
        float x2 = image.cols - 100;
        float y2 = ((-currLine.a * x2) - currLine.c) / currLine.b;

        Point displayPoint1 = Point(center.x + (x1 / scaleFactor), center.y - (y1 /scaleFactor));
        Point displayPoint2 = Point(center.x + (x2 / scaleFactor), center.y - (y2 / scaleFactor));
        Scalar pointColour = Scalar(0, 255, 0);
        line(image, displayPoint1, displayPoint2, pointColour);
    }
    imshow("RANSAC", image);
    waitKey(1);
    return;
}
#endif