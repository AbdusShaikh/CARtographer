#include "EkfSlam.h"
EkfSlam::EkfSlam(){};
EkfSlam::~EkfSlam(){};
// STEPS:
//  Predict:
//      - Predict robot pose based on prediction model using wheel encoder outputs (control terms)
//      - Update prediction model jacobian with new control terms (A)
//      - Update process noise matrix with new control terms
//      - Calculate new covariance for robot position (Only top-left 3x3 sub-matrix of covariance) 
//          - P = A * P * A + Q
//      - Update top 3 rows of covariance matrix (Robot to feature cross-correlation)
//
//  Data association (Called in the middle of update step? (when iterating over existing landmarks?)):
//      - For each incoming landmark
//          - Find the closest existing landmark (in it's predicted position h) to this by Euclidean distance
//          - Only associate this landmark if it passes the following check
//              - v^T * S^-1 * v <= lambda
//                  - v = innovation (difference)
//                  - S = innovation covaraince
//                  - lambda = some constant validation gate value
//
// Update (Re-observed landmarks):
//      - For each existing landmark
//          - Predict its position relative to the new robot pose
//              - Convert coordinates to polar in parallel
//          - Compute new Jacobian(H) (Measurement model jacobian)
//          - Update measurement noise matrix to use current landmarks range and bearing values
//          - Calculate innovation covariance of this landmark S
//          - Use Innovation covariance S to find closest incoming landmark to this existing landmark
//          - If a matching landmark is found, update state vector
//              - If no good match is found, do not update state vector with this information



//TODO:
// Initialize all matrices and vectors
//  - True and predicted state
//  - True and predicted covariance
//  - 
int EkfSlam::init(){
    trueState_x = cv::Mat(3, 1, CV_32F, {0, 0, 0 }); // Initial robot pose is at "origin"
    predictedState_x = cv::Mat(3, 1, CV_32F, {0, 0, 0 }); // Initial robot pose is at "origin"

    trueCovariance_P = cv::Mat::zeros(3,1, CV_32F);
    predictedCovariance_P = cv::Mat::zeros(3,1, CV_32F);

    return EXIT_SUCCESS;
}

// ------------------------
// Main Algorithm functions
// ------------------------

// "Main" function of Kalman Filter. Executed predict and update steps
cv::Mat EkfSlam::step(vector<scanDot> measurements, OdometryDataContainer controlInputs){
    m_controlInputs = controlInputs;
    m_measurements = measurements;
    predict();
    update();
    return trueState_x;
}

// Predict step of Kalman Filtering
void EkfSlam::predict(){

    float carWheelBase = m_controlInputs.carWheelBase;
    float lDist_mm = m_controlInputs.leftWheelDist;
    float rDist_mm = m_controlInputs.rightWheelDist;
    float currX = trueState_x.at<float>(0,0);
    float currY = trueState_x.at<float>(1,0);
    float currTheta = trueState_x.at<float>(2,0);


    float avgDistTravelled_mm = (lDist_mm + rDist_mm) / 2.0f;
    float wheelDiff_mm = ((rDist_mm - lDist_mm));

    float dx_mm = (avgDistTravelled_mm * cos(currTheta + (wheelDiff_mm / (2.0f * carWheelBase))));
    float dy_mm = (avgDistTravelled_mm * sin(currTheta + (wheelDiff_mm / (2.0f * carWheelBase))));
    float dTheta = wheelDiff_mm / carWheelBase;

    predictStateVec(currX + dx_mm, currY + dy_mm, currTheta + dTheta);
    updateTransitionJacobian(dx_mm, dy_mm);
    updateProcessNoise(dx_mm, dy_mm, dTheta);

    predictCovarianceMat();
    // predictMeasurements();
 }

// Update step of Kalman Filtering
void EkfSlam::update(){
    
    // cv::Mat innovationCovariance = (observation_H * predictedCovariance_P * observation_H.t()) + measurementNoise_R;
    // kalmanGain_K = predictedCovariance_P * observation_H * (innovationCovariance.inv());
    // trueState_x = predictedState_x + kalmanGain_K * (measurement_z - predictedMeasurement_z);
    // trueCovariance_P = predictedCovariance_P - (kalmanGain_K * innovationCovariance * kalmanGain_K.t());
};

// --------------------------
// Algorithm helper functions
// --------------------------

// Prediction helpers
// Predicts where the landmarks in the previous state will appear now in robot frame
void EkfSlam::predictMeasurements(){
    assert((predictedState_x.rows - 3) % 2 == 0); // One measurement is given by 2 data points (distance, angle)
    float robotX = predictedState_x.at<float>(0,0);
    float robotY = predictedState_x.at<float>(1,0);
    float robotTheta = predictedState_x.at<float>(2,0);
    
    predictedMeasurement_z.release();
    for (int i = 3; i < (int) predictedState_x.rows; i += 2){ // Start at the first measurement. (First 3 rows of state vector are robot pose)
        float measurementR = predictedState_x.at<float>(i, 0);
        float measurementTheta = predictedState_x.at<float>(i + 1, 0);

        float predictedR = measurementR - ((robotX * cos(measurementTheta)) + (robotY * sin(measurementTheta)));
        float predictedTheta = measurementR - robotTheta;

        cv::Mat currPredictedMeasurement = cv::Mat(2, 1, CV_32F, {predictedR, predictedTheta});

        predictedMeasurement_z.push_back(currPredictedMeasurement);
    }
};

void EkfSlam::updateTransitionJacobian(float dx_mm, float dy_mm){
    stateTransitionJacobian_A = cv::Mat::eye(3,3, CV_32F);
    stateTransitionJacobian_A.at<float>(0, 2) = -dy_mm;
    stateTransitionJacobian_A.at<float>(1, 2) = dx_mm;
}

void EkfSlam::predictStateVec(float predictedX, float predictedY, float predictedTheta){
    predictedState_x.at<float>(0,0) = predictedX;
    predictedState_x.at<float>(1,0) = predictedY;
    predictedState_x.at<float>(2,0) = predictedTheta;
}

//TODO: Find proper noise measurements
void EkfSlam::updateProcessNoise(float dx_mm, float dy_mm, float dTheta){
    cv::Mat W = cv::Mat(3,1, CV_32F, {dx_mm, dy_mm, dTheta});
    // 3x3 matrix with odometryError on the diagnol
    cv::Mat C = cv::Mat::eye(3,3, CV_32F) * cv::Mat(3,1, CV_32F, {m_odometryError, m_odometryError, m_odometryError});
    processNoise_Q = W * C * W.t();
}

// Update robot pose covariance and robot to feature correlations
void EkfSlam::predictCovarianceMat(){
    // Update the robot pose covariance submatrix
    cv::Mat covarianceTopLeft = predictedCovariance_P(cv::Rect(0, 0, 3, 3));
    predictedCovariance_P(cv::Rect(0, 0, 3, 3)) = (stateTransitionJacobian_A * covarianceTopLeft.clone() *  stateTransitionJacobian_A) + processNoise_Q;

    // Update the robot to feature correlations
    int covarianceCols = predictedCovariance_P.cols;
    cv::Mat covarianceTop3Rows = predictedCovariance_P(cv::Rect(0, 0, covarianceCols, 3));
    covarianceTop3Rows = stateTransitionJacobian_A * covarianceTop3Rows.clone();

}
// -----------------
// Utility functions
// -----------------

// TODO: Probably remove
// Creates a column vector (input vector) from vehicle movement data. Pads rows to allow addition with state vector
void EkfSlam::createInputsVec(OdometryDataContainer controlInputs){
    input_u = cv::Mat(3, 1, CV_32F, {controlInputs.dx, controlInputs.dy, controlInputs.dTheta});
    int additionalRowsNeeded = trueState_x.rows - input_u.rows;
    assert(additionalRowsNeeded >= 0);
    cv::Mat pad = cv::Mat::zeros(additionalRowsNeeded, 1, CV_32F);
    input_u.push_back(pad);
}

// Converts a list of measurements into a column vector
void EkfSlam::createMeasurementsVec(vector<scanDot> measurements){
    measurement_z.release();
    for (int i = 0; i < (int) measurements.size(); i ++){
        // Endpoints of current landmark line
        cv::Mat currMeasurement = cv::Mat(2, 1, CV_32F, {measurements[i].dist, measurements[i].angle});
        measurement_z.push_back(currMeasurement);
    }
}

