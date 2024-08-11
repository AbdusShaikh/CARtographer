#include "SplitAndMerge.h"

CartesianCoord SplitAndMerge::polarToCartesian(scanDot polarPoint){
    CartesianCoord cartesianPoint;
    cartesianPoint.x = polarPoint.dist * cos(polarPoint.angle);
    cartesianPoint.y = polarPoint.dist * sin(polarPoint.angle);
    return cartesianPoint;
}

CartesianCoord SplitAndMerge::normalizeVector(CartesianCoord vector){
    float magnitude = sqrt((vector.x * vector.x) + (vector.y + vector.y));
    CartesianCoord normalizedVector;
    normalizedVector.x = vector.x / magnitude;
    normalizedVector.y = vector.y / magnitude;
    return normalizedVector;
}

CartesianCoord SplitAndMerge::cartesianPointsToVector(CartesianCoord vector1, CartesianCoord vector2){
    CartesianCoord resVector;
    resVector.x = vector2.x - vector1.x;
    resVector.y = vector2.y - vector1.y;
    return resVector;
}

float SplitAndMerge::dotProduct(CartesianCoord vector1, CartesianCoord vector2){
    return (vector1.x * vector2.x) + (vector1.y * vector2.y);
}

/*
Based on the formula for the distance from a test point (x0, y0) to a line defined by two points (x1, y1) and (x2, y2)
Distance (linePoint1 = (x1, y1), linePoint2 = (x2, y2), testPoint = (x0, y0)):
    |((y2 - y1) * x0) - ((x2 - x1) * y0) + (x2 * y1) - (y2 * x1)|
    -------------------------------------------------------
                sqrt((y2 - y1)^2 + (x2 - x1)^2)
*/ 
float SplitAndMerge::pointToLineDist(scanDot linePoint1, scanDot linePoint2, scanDot testPoint){
    CartesianCoord linePoint1Cartesian = polarToCartesian(linePoint1); 
    CartesianCoord linePoint2Cartesian = polarToCartesian(linePoint2); 
    CartesianCoord testPointCartesian = polarToCartesian(testPoint);

    float lineXDiff = linePoint2Cartesian.x - linePoint1Cartesian.x;
    float lineYDiff = linePoint2Cartesian.y - linePoint1Cartesian.y;

    float numerator = fabs((lineYDiff * testPointCartesian.x) - (lineXDiff * testPointCartesian.y) + (linePoint2Cartesian.x * linePoint1Cartesian.y) - (linePoint2Cartesian.y * linePoint1Cartesian.x));
    float denominator = sqrt((lineYDiff * lineYDiff) + (lineXDiff * lineXDiff));

    return numerator / denominator;
}


/*
Visualization function to display the lines extracted by the algorithm
*/
void SplitAndMerge::displayExtractedLines(vector<vector<scanDot>> mergedSegments){
    int screenWidth = 800;
    int screenHeight = 800;

    Mat image = Mat::zeros(screenHeight, screenWidth, CV_8UC3);
    Point center = Point(screenHeight / 2, screenWidth / 2);
    circle(image, center, 5, Scalar(0, 255,0));
    for (int i = 0; i < (int) mergedSegments.size(); i++){
        scanDot lineStart = mergedSegments[i].front();
        scanDot lineEnd = mergedSegments[i].back();
        if (!lineStart.dist || !lineEnd.dist) continue;

        int scaleFactor = 20;
        Point lineStartCartesian = Point(center.x + ((lineStart.dist / scaleFactor) * cos(lineStart.angle)), center.y - ((lineStart.dist / scaleFactor) * sin(lineStart.angle)));
        Point lineEndCartesian = Point(center.x + ((lineEnd.dist / scaleFactor) * cos(lineEnd.angle)), center.y - ((lineEnd.dist / scaleFactor) * sin(lineEnd.angle)));
        cv::line(image, lineStartCartesian, lineEndCartesian, Scalar(0,255,0), 2);
    }
    cv::imshow("Extracted Lidar lines", image);
    cv::waitKey(1);
    return;
}

/*
Take 'rawPoints' and seperate it into a vector of vectors where each inner vector is a set of points that can have a 
line fitted through them satisfying some miminum threshold
*/
vector<vector<scanDot>> SplitAndMerge::split(vector<scanDot> rawPoints, float distThreshold){
    if (rawPoints.size() < 2){
        return {rawPoints};
    }

    float maxDist = 0.0f;
    int maxIdx = -1;

    scanDot lineStartPoint = rawPoints.front();
    scanDot lineEndPoint = rawPoints.back();
    for (int i = 1; i < (int) rawPoints.size() - 1; i ++){
        float currPerpDist = pointToLineDist(lineStartPoint, lineEndPoint, rawPoints[i]);
        if (currPerpDist > maxDist){
            maxDist = currPerpDist;
            maxIdx = i;
        }
    }

    if (maxDist > distThreshold){
        vector<scanDot> leftPoints = vector<scanDot>(rawPoints.begin(), rawPoints.begin() + (maxIdx));
        vector<scanDot> rightPoints = vector<scanDot>(rawPoints.begin() + maxIdx + 1, rawPoints.end());
        vector<vector<scanDot>> leftSegments = split(leftPoints, distThreshold);
        vector<vector<scanDot>> rightSegments = split(rightPoints, distThreshold);

        assert((leftSegments.front().front()).angle <= (leftSegments.back().back()).angle);
        assert((rightSegments.front().front()).angle <= (rightSegments.back().back()).angle);

        vector<vector<scanDot>> combinedSegments = leftSegments;
        combinedSegments.insert(combinedSegments.end(), rightSegments.begin(), rightSegments.end()); // Combine the segments into one vector
        return combinedSegments;
    }
    else {
        return {rawPoints};
    }
}

/*
Combine line segments whose vectors make an angle that is within some threshold indicating their parallellness.
*/
vector<vector<scanDot>> SplitAndMerge::merge(vector<vector<scanDot>> splitSegments, float angThreshold){
    vector<vector<scanDot>> mergedSegments;
    vector<scanDot> currSegment = splitSegments.front();

    for (int i = 1; i < (int) splitSegments.size(); i ++){
        vector<scanDot> nextSegment = splitSegments[i];

        CartesianCoord currSegmentStart = polarToCartesian(currSegment.front());
        CartesianCoord currSegmentEnd = polarToCartesian(currSegment.back());
        CartesianCoord currSegmentVectorNormalized = normalizeVector(cartesianPointsToVector(currSegmentStart, currSegmentEnd));
        
        CartesianCoord nextSegmentStart = polarToCartesian(nextSegment.front());
        CartesianCoord nextSegmentEnd = polarToCartesian(nextSegment.back());
        CartesianCoord nextSegmentVectorNormalized = normalizeVector(cartesianPointsToVector(nextSegmentStart, nextSegmentEnd));

        float dotProd = dotProduct(currSegmentVectorNormalized, nextSegmentVectorNormalized);
        if (fabs(1.0f - dotProd) < angThreshold){
            // Only endpoints
            // currSegment = {currSegment.front(), nextSegment.back()};
            // Complete line
            currSegment.insert(currSegment.end(), nextSegment.begin(), nextSegment.end());
        } 
        else {
            mergedSegments.push_back(currSegment);
            currSegment = nextSegment;
        }
    }
    mergedSegments.push_back(currSegment);

    return mergedSegments;
}

/*
Filtering function to remove segments based on the following rules
    - Minimum line length
    - 
*/
vector<vector<scanDot>> SplitAndMerge::filterSegments(vector<vector<scanDot>> mergedSegments, int minLineLength){
    vector<vector<scanDot>> filteredSegments;
    for (int i = 0; i < (int) mergedSegments.size(); i ++){
        if ((int) mergedSegments[i].size() < minLineLength){
            continue;
        }
        vector<scanDot> currSegmentEndpoints = {mergedSegments[i].front(), mergedSegments[i].back()};
        // filteredSegments.push_back(mergedSegments[i]);
        filteredSegments.push_back(currSegmentEndpoints);
    }
    return filteredSegments;
}

vector<vector<scanDot>>  SplitAndMerge::splitAndMerge(vector<scanDot> rawPoints, float distThreshold, float angThreshold, int minLineLength){
    vector<vector<scanDot>> segments = split(rawPoints, distThreshold);
    vector<vector<scanDot>> mergedSegments = merge(segments, angThreshold);
    vector<vector<scanDot>> filteredSegments = filterSegments(mergedSegments, minLineLength);


#if DISPLAY_EXTRACTED_LINES_SPLITANDMERGE
    displayExtractedLines(filteredSegments);
#endif

    return filteredSegments;
}