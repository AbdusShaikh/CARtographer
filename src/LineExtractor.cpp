#include "LineExtractor.h"
// TODO:
//  - Ensure scanDot.angle is coming in as Radians


CartesianCoord LineExtractor::polarToCartesian(scanDot polarPoint){
    CartesianCoord cartesianPoint;
    cartesianPoint.x = polarPoint.dist * cos(polarPoint.angle);
    cartesianPoint.y = polarPoint.dist * sin(polarPoint.angle);
    return cartesianPoint;
}

CartesianCoord LineExtractor::cartesianPointsToVector(CartesianCoord vector1, CartesianCoord vector2){
    CartesianCoord resVector;
    resVector.x = vector2.x - vector1.x;
    resVector.y = vector2.y - vector1.y;
    return resVector;
}

float LineExtractor::dotProduct(CartesianCoord vector1, CartesianCoord vector2){
    return (vector1.x * vector2.x) + (vector1.y * vector2.y);
}

/*
Based on the formula for the distance from a test point (x0, y0) to a line defined by two points (x1, y1) and (x2, y2)
Distance (linePoint1 = (x1, y1), linePoint2 = (x2, y2), testPoint = (x0, y0)):
    |((y2 - y1) * x0) - ((x2 - x1) * y0) + (x2 * y1) - (y2 * x1)|
    -------------------------------------------------------
                sqrt((y2 - y1)^2 + (x2 - x1)^2)
*/ 
float LineExtractor::pointToLineDist(scanDot linePoint1, scanDot linePoint2, scanDot testPoint){
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
Take 'rawPoints' and seperate it into a vector of vectors where each inner vector is a set of points that can have a 
line fitted through them satisfying some miminum threshold
*/
vector<vector<scanDot>> LineExtractor::split(vector<scanDot> rawPoints, float distThreshold){
    if (rawPoints.size() <= 2){
        return {rawPoints};
    }

    float maxDist = 0.0f;
    int maxIdx = -1;

    scanDot lineStartPoint = rawPoints.front();
    scanDot lineEndPoint = rawPoints.back();
    for (int i = 1; i < rawPoints.size() - 1; i ++){
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
vector<vector<scanDot>> LineExtractor::merge(vector<vector<scanDot>> splitSegments, float angThreshold){
    vector<vector<scanDot>> mergedSegments;
    vector<scanDot> currSegment = splitSegments.front();

    for (int i = 1; i < splitSegments.size(); i ++){
        vector<scanDot> nextSegment = splitSegments[i];

        CartesianCoord currSegmentStart = polarToCartesian(currSegment.front());
        CartesianCoord currSegmentEnd = polarToCartesian(currSegment.back());
        CartesianCoord currSegmentVectorNormalized = normalizeVector(cartesianPointsToVector(currSegmentStart, currSegmentEnd));
        
        CartesianCoord nextSegmentStart = polarToCartesian(nextSegment.front());
        CartesianCoord nextSegmentEnd = polarToCartesian(nextSegment.back());
        CartesianCoord nextSegmentVectorNormalized = normalizeVector(cartesianPointsToVector(nextSegmentStart, nextSegmentEnd));

        float dotProd = dotProduct(currSegmentVectorNormalized, nextSegmentVectorNormalized);
        if (fabs(1.0f - dotProd) < angThreshold){
            currSegment.insert(currSegment.end(), nextSegment.begin(), nextSegment.end()); // Combine the segments into one vector
        } 
        else {
            mergedSegments.push_back(currSegment);
            currSegment = nextSegment;
        }
    }
    if (mergedSegments.empty()){
        mergedSegments.push_back(currSegment);
    }

    return mergedSegments;
}

vector<vector<scanDot>>  LineExtractor::splitAndMerge(vector<scanDot> rawPoints, float distThreshold, float angThreshold){
    vector<vector<scanDot>> segments = split(rawPoints, distThreshold);
    vector<vector<scanDot>> mergedSegments = merge(segments, angThreshold);

    return mergedSegments;
}