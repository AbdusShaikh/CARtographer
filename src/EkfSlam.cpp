#include "EkfSlam.h"
EkfSlam::EkfSlam(){};
EkfSlam::~EkfSlam(){};

//TODO:
// Initialize all matrices and vectors
int EkfSlam::init(){
    trueState_x = cv::Mat(3, 1, CV_32F, {0, 0, 0 }); // Initial robot pose is at "origin"
    return EXIT_SUCCESS;
}

// ------------------------
// Main Algorithm functions
// ------------------------

// "Main" function of Kalman Filter. Executed predict and update steps
cv::Mat EkfSlam::step(vector<scanDot> measurements, OdometryDataContainer controlInputs){
    createInputsVec(controlInputs);
    createMeasurementsVec(measurements);
    predict();
    update();
    return trueState_x;
}

// Predict step of Kalman Filtering
void EkfSlam::predict(){
    predictedState_x = trueState_x + input_u;
    predictMeasurements();
    associateMeasurements(1.0f);
    // Identity matrix
    cv::Mat stateTransitionJacobian = cv::Mat::eye(trueState_x.rows, trueState_x.cols, CV_32F);
    // (F_y * P_(t-1) * (F_y)^T) + (F_u * Q_t * (F_u)^T))
    predictedEstimateCovariance_P = (stateTransitionJacobian * trueEstimateCovariance_P * stateTransitionJacobian.t()) + (stateTransitionJacobian * processNoiseCovariance_Q * stateTransitionJacobian.t());
}

// Update step of Kalman Filtering
void EkfSlam::update(){
    cv::Mat innovationCovariance = (observation_H * predictedEstimateCovariance_P * observation_H.t()) + measurementCovariance_R;
    kalmanGain_K = predictedEstimateCovariance_P * observation_H * (innovationCovariance.inv());
    trueState_x = predictedState_x + kalmanGain_K * (measurement_z - predictedMeasurement_z);
    trueEstimateCovariance_P = predictedEstimateCovariance_P - (kalmanGain_K * innovationCovariance * kalmanGain_K.t());
};

// --------------------------
// Algorithm helper functions
// --------------------------

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

//TODO: 
// - Consider Mahalanobis distance over euclidean
// - Identify new landmarks and add them to state/measurements vector

// Line up (associate) measurements_z with predictedMeasurements_z to properly execute update step in Kalman Filter
void EkfSlam::associateMeasurements(float distThreshold){
    cv::Mat associatedMeasurements = cv::Mat::zeros(predictedMeasurement_z.rows, predictedMeasurement_z.cols, CV_32F);
    
    int predictedMeasurementRows = predictedMeasurement_z.rows; // This number may change if we encounter new landmarks so we want to use the initial valie
    for (int i = 0; i < (int) measurement_z.rows; i += 2){
        int bestMatchIdx = -1;
        float minDist = INFINITY;
        float r1 = measurement_z.at<float>(i, 0);
        float r1sqrd = r1 * r1;
        float theta1 = measurement_z.at<float>(i + 1, 0);
        for (int j = 0; j < predictedMeasurementRows; j += 2){
            float r2 = predictedMeasurement_z.at<float>(j, 0);
            float theta2 = predictedMeasurement_z.at<float>(j + 1, 0);
            float currDistance = sqrt(r1sqrd + (r2 * r2) - (2 * r1 * r2 * cos(theta2 - theta1)));
            if (currDistance < minDist){
                minDist = currDistance;
                bestMatchIdx = j;
            }
        }
        if (minDist < distThreshold){ // Found a good neighbour (successfully associated)
            associatedMeasurements.at<float>(bestMatchIdx, 0) = measurement_z.at<float>(i, 0 );
            associatedMeasurements.at<float>(bestMatchIdx + 1, 0) = measurement_z.at<float>(i + 1, 0 );
        }
        else { // Could not find good neighbour. Likely a new landmark
            // Convert measurement coordinates from local to global coordinates 
            float robotX = predictedState_x.at<float>(0, 0);
            float robotY = predictedState_x.at<float>(1, 0);
            float robotTheta = predictedState_x.at<float>(2, 0);

            float measurementGlobalX = robotX + (r1 * cos(robotTheta + theta1)); // robotX + measurementX
            float measurementGlobalY = robotY + (r1 * sin(robotTheta + theta1)); // robotY + measurementY

            float globalR = sqrt((measurementGlobalX * measurementGlobalX) + (measurementGlobalY * measurementGlobalY));
            float globalTheta = robotTheta + theta1; // Robot pose theta + measurement theta = global measurement theta
            // Add this new landmark to the state vector
            cv::Mat newLandmarkGlobal = cv::Mat(2, 1, CV_32F, {globalR, globalTheta});
            cv::Mat newLandMarkLocal = cv::Mat(2, 1, CV_32F, {r1, theta1});
            predictedMeasurement_z.push_back(newLandMarkLocal);
            associatedMeasurements.push_back(newLandMarkLocal);
            // TODO: What happens to the Measurement Model Jacobian if the 
            predictedState_x.push_back(newLandmarkGlobal); // Add this landmark to the state vector. It will be updated in subsequent timesteps
        }
    }
    measurement_z = associatedMeasurements;
}

// -----------------
// Utility functions
// -----------------

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

// Creates the jacobian matrix of the measurement model. 
// Executed after data association because state and measurement vectors change size during that step.
void EkfSlam::createObservationJacobian(){
    observation_H = cv::Mat::zeros(measurement_z.rows, predictedState_x.rows, CV_32F);

    for (int i = 0; i < (int) measurement_z.rows; i += 2){
        // This landmark was in the state vector but not detected this timestep (not associated). Do not update it
        if ((measurement_z.at<float>(i, 0) == measurement_z.at<float>(i, 0)) && (measurement_z.at<float>(i, 0) == 0.0f)){ 

        }
        // This measurement was detected before and detected again. Update it
        else { 

        }
    }
}

// Expose the state to outside world
cv::Mat EkfSlam::getState(){
    return trueState_x;
}