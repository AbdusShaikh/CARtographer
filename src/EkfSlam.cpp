#include "EkfSlam.h"


EkfSlam::EkfSlam(){};
EkfSlam::~EkfSlam(){};

//TODO:
// Initialize all matrices and vectors
int EkfSlam::init(){
    return EXIT_SUCCESS;
}


cv::Mat EkfSlam::step(vector<vector<scanDot>> measurements){
    predict();
    update(createMeasurementsMat(measurements));
    return trueState_x;
}

void EkfSlam::predict(){
    // TODO:
    // input_u comes as a 3 row 1 col vector. We need to pad the bottom rows to do this addition
    predictedState_x = trueState_x + input_u;
    cv::Mat stateTransitionJacobian = cv::Mat::eye(trueState_x.rows, trueState_x.cols, CV_32F);
    // (F_y * P_(t-1) * (F_y)^T) + (F_u * Q_t * (F_u)^T))
    predictedEstimateCovariance_P = (stateTransitionJacobian * trueEstimateCovariance_P * stateTransitionJacobian.t()) + (stateTransitionJacobian * processNoiseCovariance_Q * stateTransitionJacobian.t());
}

void EkfSlam::update(cv::Mat measurements){
    cv::Mat innovationCovariance = (observation_H * predictedEstimateCovariance_P * observation_H.t()) + measurementCovariance_R;
    kalmanGain_K = predictedEstimateCovariance_P * observation_H * (innovationCovariance.inv());
    measurement_z = measurements;
    trueState_x = predictedState_x + kalmanGain_K * (measurement_z - predictMeasurements());
    trueEstimateCovariance_P = predictedEstimateCovariance_P - (kalmanGain_K * innovationCovariance * kalmanGain_K.t());

};

// TODO:
cv::Mat EkfSlam::predictMeasurements(){
    return cv::Mat::eye(1, 1, CV_32F);
};

// TODO:
cv::Mat EkfSlam::createMeasurementsMat(vector<vector<scanDot>> measurements){
    (void) measurements;
    return cv::Mat::eye(1, 1, CV_32F);
}

cv::Mat EkfSlam::getState(){
    return trueState_x;
}