#include "EkfSlam.h"
// TODO:
//  - Are measurements in Polar or Cartesian coordinates? Unify them
//      - Robot pose: cartesian + rotation angle
//      - Landmarks/Measurements: Polar

EkfSlam::EkfSlam(){};
EkfSlam::~EkfSlam(){};

//TODO:
// Initialize all matrices and vectors
int EkfSlam::init(){
    return EXIT_SUCCESS;
}


cv::Mat EkfSlam::step(vector<vector<scanDot>> measurements){
    predict();
    update(measurements);
    return trueState_x;
}

void EkfSlam::predict(){
    padInput();
    predictedState_x = trueState_x + input_u;
    // Identity matrix
    cv::Mat stateTransitionJacobian = cv::Mat::eye(trueState_x.rows, trueState_x.cols, CV_32F);
    // (F_y * P_(t-1) * (F_y)^T) + (F_u * Q_t * (F_u)^T))
    predictedEstimateCovariance_P = (stateTransitionJacobian * trueEstimateCovariance_P * stateTransitionJacobian.t()) + (stateTransitionJacobian * processNoiseCovariance_Q * stateTransitionJacobian.t());
}

void EkfSlam::update(vector<vector<scanDot>> measurements){
    createMeasurementsMat(measurements);
    predictMeasurements();
    cv::Mat innovationCovariance = (observation_H * predictedEstimateCovariance_P * observation_H.t()) + measurementCovariance_R;
    kalmanGain_K = predictedEstimateCovariance_P * observation_H * (innovationCovariance.inv());
    trueState_x = predictedState_x + kalmanGain_K * (measurement_z - predictedMeasurement_z);
    trueEstimateCovariance_P = predictedEstimateCovariance_P - (kalmanGain_K * innovationCovariance * kalmanGain_K.t());

};

// TODO:
// Predicts where the landmarks in the previous state will appear now in robot frams
void EkfSlam::predictMeasurements(){
    assert((predictedState_x.rows - 3) % 2 == 0); // One measurement is given by 2 data points (distance, angle)
    float robotX = predictedState_x.at<float>(0,0);
    float robotY = predictedState_x.at<float>(1,0);
    float robotTheta = predictedState_x.at<float>(2,0);
    
    predictedMeasurement_z.release();
    for (int i = 3; i < (int) predictedState_x.rows; i += 2){ // Start at the first measurement. (First 3 rows of state vector are robot pose)
        float measurementR = predictedState_x.at<float>(i, 0);
        float measurementTheta = predictedState_x.at<float>(i + 1, 0);

        float predictedR = measurementR - robotTheta;
        float predictedTheta = measurementR - ((robotX * cos(measurementTheta)) + (robotY * sin(measurementTheta)));

        cv::Mat currPredictedMeasurement = cv::Mat(2, 1, CV_32F, {predictedR, predictedTheta});

        predictedMeasurement_z.push_back(currPredictedMeasurement);
    }
};

void EkfSlam::padInput(){
    int additionalRowsNeeded = trueState_x.rows - input_u.rows;
    assert(additionalRowsNeeded >= 0);
    cv::Mat pad = cv::Mat::zeros(additionalRowsNeeded, 1, CV_32F);
    input_u.push_back(pad);
}

void EkfSlam::createMeasurementsMat(vector<vector<scanDot>> measurements){
    measurement_z.release();
    for (int i = 0; i < (int) measurements.size(); i ++){
        // Endpoints of current landmark line
        cv::Mat currMeasurement = cv::Mat(2, 1, CV_32F, {measurements[i].front().dist, measurements[i].front().angle, measurements[i].back().dist, measurements[i].back().angle});
        measurement_z.push_back(currMeasurement);
    }
}

//TODO:
// Line up (associate) measurements_z with predictedMeasurements_z to properly execute update step in Kalman Filter
void EkfSlam::associateMeasurements(){
    cv::Mat associatedMeasurements = cv::Mat::zeros(predictedMeasurement_z.rows, predictedMeasurement_z.cols, CV_32F);

    for (int i = 0; i < (int) measurement_z.rows; i += 2){
        int bestMatchIdx = -1;
        float minDist = INFINITY;
        float r1 = measurement_z.at<float>(i, 0);
        float r1sqrd = r1 * r1;
        float theta1 = measurement_z.at<float>(i + 1, 0);
        for (int j = 0; j < (int) predictedMeasurement_z.rows; j += 2){
            float r2 = predictedMeasurement_z.at<float>(j, 0);
            float theta2 = predictedMeasurement_z.at<float>(j + 1, 0);
            float currDistance = sqrt(r1sqrd + (r2 * r2) - (2 * r1 * r2 * cos(theta2 - theta1)));
            if (currDistance < minDist){
                minDist = currDistance;
                bestMatchIdx = j;
            }
        }
        assert (bestMatchIdx != -1);
        assert (minDist < INFINITY);
        associatedMeasurements.at<float>(bestMatchIdx, 0) = measurement_z.at<float>(i, 0 );
        associatedMeasurements.at<float>(bestMatchIdx + 1, 0) = measurement_z.at<float>(i + 1, 0 );
    }
    measurement_z = associatedMeasurements;
}

cv::Mat EkfSlam::getState(){
    return trueState_x;
}