#include "LandmarkManager.h"

LandmarkManager::LandmarkManager(){};
LandmarkManager::~LandmarkManager(){};

vector<scanDot> LandmarkManager::step(vector<scanDot> measurements, float predictedX, float predictedY, float predictedTheta){
    load(measurements, predictedX, predictedY, predictedTheta);
    updateStatuses();
    return goodLandmarks;
}

// TODO: Use predicted points to compare with incoming measurements

// Process new measurements to associate them with previously seen landmarks or create new ones
void LandmarkManager::load(vector<scanDot> measurements, float predictedX, float predictedY, float predictedTheta){
    // Does this landmark already exist?
    //  - If yes: increase observation count
    //  - If no: Initialize new landmark
    for (int i = 0; i < (int) measurements.size(); i++){
        float r1 = measurements[i].dist;
        float t1 = measurements[i].angle;
        float bestMahDist = INFINITY; // Mahalanobis Distance
        float bestIdx = -1;
        for (int j = 0; j < (int) observedLandmarks.size(); j++){
            float r2 = observedLandmarks[j].point.dist;
            float t2 = observedLandmarks[j].point.angle;

            float xDiff = (r2 * cos(t2)) - predictedX;
            float yDiff = (r2 * sin(t2)) - predictedY;
            float expectedRange = sqrt((xDiff * xDiff) + (yDiff * yDiff)); // Expected landmark range based on the predicted pose
            float expectedBearing = atan2(yDiff, xDiff) - predictedTheta; // Expected landmark bearing based on the predicted pose
            
            // TODO: REMOVE HARDCODED VALUES
            float mahDist = sqrt((((expectedRange - r1) * (expectedRange - r1)) / 135.48) + (((expectedBearing - t1) * (expectedBearing - t1)) / 1.79e-5) );
            if (mahDist < bestMahDist){
                bestMahDist = mahDist;
                bestIdx = j;
            }
        }
        // Based on Chi-Square distribution thresholding values found here https://www.itl.nist.gov/div898/handbook/eda/section3/eda3674.htm
        if (bestMahDist <= 4.6){ 
            // Found association
            observedLandmarks[bestIdx].point.angle = t1;
            observedLandmarks[bestIdx].point.dist = r1;
            observedLandmarks[bestIdx].recentlyObserved = true;
        }
        // No association found. Create a new landmark
        else {
            Landmark newLandmark;
            newLandmark.observationCount = 0;
            newLandmark.point.dist = r1;
            newLandmark.point.angle = t1;
            newLandmark.status = unconfirmed;
            newLandmark.recentlyObserved = true;
            observedLandmarks.push_back(newLandmark);
        }
    }
}

// Categorize landmarks between good and bad. Bad landmarks are removed
void LandmarkManager::updateStatuses(){
    goodLandmarks.clear();
    for (int i = 0; i < (int) observedLandmarks.size(); i++){
        if (!observedLandmarks[i].recentlyObserved && observedLandmarks[i].status == unconfirmed){
            observedLandmarks[i].observationCount --;
            if (observedLandmarks[i].observationCount <= 0){
                // Remove this landmark 
                swap(observedLandmarks[i], observedLandmarks.back());
                observedLandmarks.pop_back();
            }
        }
        else {
            observedLandmarks[i].observationCount ++;
            observedLandmarks[i].recentlyObserved = false;
            if (observedLandmarks[i].observationCount >= confirmationCount){ 
                observedLandmarks[i].status = confirmed;
                goodLandmarks.push_back(observedLandmarks[i].point);
            }
        }
    }
}
