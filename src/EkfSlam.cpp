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
    // predict();
    createMeasurementsMat(measurements);
    // update();
    return trueState_x;
}

void EkfSlam::predict(){
    padInput();
    predictedState_x = trueState_x + input_u;
    cv::Mat stateTransitionJacobian = cv::Mat::eye(trueState_x.rows, trueState_x.cols, CV_32F);
    // (F_y * P_(t-1) * (F_y)^T) + (F_u * Q_t * (F_u)^T))
    predictedEstimateCovariance_P = (stateTransitionJacobian * trueEstimateCovariance_P * stateTransitionJacobian.t()) + (stateTransitionJacobian * processNoiseCovariance_Q * stateTransitionJacobian.t());
}

void EkfSlam::update(){
    cv::Mat innovationCovariance = (observation_H * predictedEstimateCovariance_P * observation_H.t()) + measurementCovariance_R;
    kalmanGain_K = predictedEstimateCovariance_P * observation_H * (innovationCovariance.inv());
    trueState_x = predictedState_x + kalmanGain_K * (measurement_z - predictMeasurements());
    trueEstimateCovariance_P = predictedEstimateCovariance_P - (kalmanGain_K * innovationCovariance * kalmanGain_K.t());

};

// TODO:
cv::Mat EkfSlam::predictMeasurements(){
    assert(measurement_z.rows % 2 == 0); // One measurement is given by 2 data points (distance, angle)
    float robotX = predictedState_x.at<float>(0,0);
    float robotY = predictedState_x.at<float>(1,0);
    float robotTheta = predictedState_x.at<float>(2,0);
    
    cv::Mat predictedMeasurements;
    for (int i = 0; i < (int) measurement_z.rows; i++){
        float measurementR = measurement_z.at<float>(i*2, 0);
        float measurementTheta = measurement_z.at<float>((i*2) + 1, 0);

        float predictedR = measurementR - robotTheta;
        float predictedTheta = measurementR - (robotX * cos(measurementTheta) + robotY * sin(measurementTheta));

        cv::Mat currMeasurementPredicted = cv::Mat(2, 1, CV_32F, {predictedR, predictedTheta});

        predictedMeasurements.push_back(currMeasurementPredicted);
    }
    return predictedMeasurements;
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

cv::Mat EkfSlam::getState(){
    return trueState_x;
}