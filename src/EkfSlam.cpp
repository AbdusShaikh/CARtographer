#include "EkfSlam.h"
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
// Adding new landmarks
//      - For each new landmark
//          - Compute its position in world coordinates using the most up-to-date state estimation
//          - Compute its measurement model jacobians (w.r.t robot state (J_xr) & w.r.t range and bearing (J_z))
//          - Compute this new landmarks co-variance (P_ll)
//              - (J_xr * P_rr * (J_xr)^T) + (J_z * R * (J_z)^T)
//          - Compute cross variance of this new landmark with all other elements in state vector before this landmark (P_lx)
//              - J_xr * (top row of covariance matrix)
//          - Add this new landmark to the bottom of the state vector
//          - Add a new row and column to the covariance matrix
//              - Bottom row: P_lx (cross variance between new landmark and other state elements)
//              - Right column: P_lx^T (cross variance but in column form)
//              - Bottom right corner: P_ll (co-variance matrix of new landmark)
//TODO:
//  - SPEED UP


EkfSlam::EkfSlam(){};
EkfSlam::~EkfSlam(){};

int EkfSlam::init(){
    state_x = Mat::zeros(3, 1, CV_32F); // Initial robot pose is at "origin"
    covariance_P = Mat::zeros(3,3, CV_32F);
    measurementNoise_R = Mat::zeros(2, 2, CV_32F);
    identity_V = Mat::eye(2,2, CV_32F);
    associatedLandmark_z = Mat::zeros(2, 1, CV_32F);

    m_associationGate = 7.0f;
    m_odometryError = 0.5f;
    // Empirically calculated
    // m_RangeVariance = 33.87f;
    m_RangeVariance = 33.87f;

    m_BearingVariance = 1.0f; // 1 radian error

    m_landmarkConfirmationCount = 40;

    return EXIT_SUCCESS;
}

// ------------------------
// Main Algorithm functions
// ------------------------

// "Main" function of Kalman Filter. Executes predict and update steps
Mat EkfSlam::step(vector<scanDot> measurements, OdometryDataContainer controlInputs){
    m_controlInputs = controlInputs;
    m_rawMeasurements = measurements;
    predict();
    update();
    addNewLandmarks();
#if DISPLAY_LANDMARKS
    displayLandmarks();
#endif
    return state_x;
}

// Predict step of Kalman Filtering
void EkfSlam::predict(){

    float carWheelBase = m_controlInputs.carWheelBase;
    float lDist_mm = m_controlInputs.leftWheelDist;
    float rDist_mm = m_controlInputs.rightWheelDist;
    float currX = state_x.at<float>(0,0);
    float currY = state_x.at<float>(1,0);
    float currTheta = state_x.at<float>(2,0);


    float avgDistTravelled_mm = (lDist_mm + rDist_mm) / 2.0f;
    float wheelDiff_mm = (rDist_mm - lDist_mm);

    float dTheta = wheelDiff_mm / carWheelBase;
    float dx_mm = avgDistTravelled_mm * cos(currTheta + (dTheta / 2));
    float dy_mm = avgDistTravelled_mm * sin(currTheta + (dTheta / 2));

    predictStateVec(currX + dx_mm, currY + dy_mm, currTheta + dTheta);
    updateTransitionJacobian(dx_mm, dy_mm);
    updateProcessNoise(dx_mm, dy_mm, dTheta);

    predictCovarianceMat();

    manageLandmarks(m_rawMeasurements);
 }

// Update step of Kalman Filtering
void EkfSlam::update(){
    for (int i = 3; i < state_x.rows; i += 2){ // Landmarks start at row 3
        // These values change as we update the state vector so we need to reinitialize them each tiem
        float rX = state_x.at<float>(0, 0);
        float rY = state_x.at<float>(1, 0);
        float rTheta = state_x.at<float>(2, 0);

        // 1.) Calculate estimated position of landmark
        float lX = state_x.at<float>(i, 0); // Current landmark's x position in world coordinates
        float lY = state_x.at<float>(i + 1, 0); // landmark's y position in world coordinates
        // Measurement model
        float xDiff = lX - rX;
        float yDiff = lY - rY;
        float expectedRange = sqrt((xDiff * xDiff) + (yDiff * yDiff)); // Expected landmark range based on the predicted pose
        float expectedBearing = atan2(yDiff, xDiff) - rTheta; // Expected landmark bearing based on the predicted pose
        Mat predictedLandmark_h = Mat::zeros(2, 1, CV_32F);
        predictedLandmark_h.at<float>(0,0) = expectedRange;
        predictedLandmark_h.at<float>(1,0) = expectedBearing;

        // 2.) Create measurement model Jacobian H
        updateMeasurementJacobian(i, rX, rY, lX, lY, expectedRange);

        // 3.) Update measurement noise matrix R
        measurementNoise_R.at<float>(0,0) = m_RangeVariance * expectedRange;
        measurementNoise_R.at<float>(1,1) = m_BearingVariance;

        // 4.) Compute innovation covariance S
        Mat innovationCovariance_S = (measurementJacobian_H * covariance_P * measurementJacobian_H.t()) + (identity_V * measurementNoise_R * identity_V.t());
        
        // 5.) Find associated landmark
        if (!associateLandmark(expectedRange, expectedBearing, innovationCovariance_S)){ // Failed to find this previously observed landmark in current reading. We want to only update reobserved landmarks
            continue;
        }
        // 6.) Compute Kalman Gain
        kalmanGain_K = covariance_P * measurementJacobian_H.t() * innovationCovariance_S.inv();

        // 7.) Update state
        state_x = state_x  + (kalmanGain_K * (associatedLandmark_z - predictedLandmark_h));
        // 8.) Update covariance
        covariance_P = covariance_P - (kalmanGain_K * innovationCovariance_S * kalmanGain_K.t());
        

    }
};


// Dynamicallly grow state vector (AKA The Map) based on observations
//TODO: Use point closest to wall from origin as state features
void EkfSlam::addNewLandmarks(){
    float robotX = state_x.at<float>(0,0);
    float robotY = state_x.at<float>(1,0);
    float robotTheta = state_x.at<float>(2,0);
    for (int i = 0; i < (int) m_goodMeasurements.size(); i++){
        // Convert landmark coordinates from robot frame to world frame ((robot_range, robot_bearing) -> (world_x, world_y))
        float landmarkR = m_goodMeasurements[i].dist;
        float landmarkTheta = m_goodMeasurements[i].angle;
        float globalTheta = robotTheta + landmarkTheta;
        float cosGlobalTheta = cos(globalTheta);
        float sinGlobalTheta = sin(globalTheta);
        float landmarkGlobalX = robotX + (landmarkR * cosGlobalTheta);
        float landmarkGlobalY = robotY + (landmarkR * sinGlobalTheta);
        

        // Compute Jacobians
        // Jacobian matrix of the inverted measurement model with respect to the robot x, y
        Mat invMeasurementPoseJacobian = Mat::eye(2, 3, CV_32F); 
        invMeasurementPoseJacobian.at<float>(0, 2) = -landmarkR * sinGlobalTheta;
        invMeasurementPoseJacobian.at<float>(1, 2) = landmarkR * cosGlobalTheta;

        // Jacobian matrix of the inverted measurement model with respect to range and bearing of new landmark
        Mat invMeasurementLandmarkJacobian = Mat::zeros(2, 2, CV_32F);
        invMeasurementLandmarkJacobian.at<float>(0,0) = cosGlobalTheta;
        invMeasurementLandmarkJacobian.at<float>(0,1) = -landmarkR * sinGlobalTheta;
        invMeasurementLandmarkJacobian.at<float>(1,0) = sinGlobalTheta;
        invMeasurementLandmarkJacobian.at<float>(1,1) = landmarkR * cosGlobalTheta;

        measurementNoise_R.at<float>(0,0) = m_RangeVariance;
        measurementNoise_R.at<float>(1,1) = m_BearingVariance ;

        // Calculate new covariance submatrix and crossvariance vectors
        Mat newLandmarkCovariance = (invMeasurementPoseJacobian * covariance_P(Rect(0, 0, 3, 3)) * invMeasurementPoseJacobian.t()) + (invMeasurementLandmarkJacobian * measurementNoise_R * invMeasurementLandmarkJacobian.t());
        Mat newLandmarkCrossVariance = invMeasurementPoseJacobian *  covariance_P(Rect(0, 0, covariance_P.cols, 3));

        // Add new landmark to state vector and its co and cross variances to the covariance matrix
        int prevRows = covariance_P.rows, prevCols = covariance_P.cols;
        // Create a new matrix with 2 more rows and columns as the previous covariance matrix
        Mat newCovariance_P = Mat::zeros(prevRows + 2, prevCols + 2, CV_32F);
        // Copy the old matrix to this new one in the top left
        Mat newCovarianceOldSubmatrix = newCovariance_P.colRange(0, prevCols).rowRange(0, prevRows);
        covariance_P.copyTo(newCovarianceOldSubmatrix);
        // Create the cross-variances for this new landmark and all previous state elements
        Mat newCovarianceLast2RowsOldSubmatrix = newCovariance_P.colRange(0, prevCols).rowRange(prevRows, prevRows + 2);
        newLandmarkCrossVariance.copyTo(newCovarianceLast2RowsOldSubmatrix);

        Mat newCovarianceLast2ColsOldSubmatrix = newCovariance_P.colRange(prevCols, prevCols + 2).rowRange(0, prevRows);
        Mat crossVarianceTranspose = newLandmarkCrossVariance.t();
        (crossVarianceTranspose).copyTo(newCovarianceLast2ColsOldSubmatrix);
        // Add the covariance for this new landmark
        Mat newCovarianceMatNewLandmarkSubmatrix = newCovariance_P.colRange(prevCols, prevCols + 2).rowRange(prevRows, prevRows + 2);
        // newCovariance_P(Rect(prevCols, prevRows, 2, 2)) = newLandmarkCovariance;
        newLandmarkCovariance.copyTo(newCovarianceMatNewLandmarkSubmatrix);

        covariance_P = newCovariance_P;

        // Add this landmark to the state vector
        state_x.push_back(landmarkGlobalX);
        state_x.push_back(landmarkGlobalY);

    }
}
// --------------------------
// Prediction helper functions
// --------------------------

// Predicts where the landmarks in the previous state will appear now in robot frame
void EkfSlam::updateTransitionJacobian(float dx_mm, float dy_mm){
    stateTransitionJacobian_A = Mat::eye(3,3, CV_32F);
    stateTransitionJacobian_A.at<float>(0, 2) = -dy_mm;
    stateTransitionJacobian_A.at<float>(1, 2) = dx_mm;
}

void EkfSlam::predictStateVec(float predictedX, float predictedY, float predictedTheta){
    state_x.at<float>(0,0) = predictedX;
    state_x.at<float>(1,0) = predictedY;
    state_x.at<float>(2,0) = predictedTheta;
}

//TODO: Find proper noise measurements
void EkfSlam::updateProcessNoise(float dx_mm, float dy_mm, float dTheta){
    Mat W = Mat::zeros(3,1, CV_32F);
    W.at<float>(0,0) = dx_mm;
    W.at<float>(1,0) = dy_mm;
    W.at<float>(2,0) = dTheta;
    
    processNoise_Q = W * m_odometryError * W.t();
}

// Update robot pose covariance and robot to feature correlations
void EkfSlam::predictCovarianceMat(){
    Mat predictedPoseCovariance = (stateTransitionJacobian_A * covariance_P(cv::Rect(0, 0, 3, 3)) *  stateTransitionJacobian_A.t()) + processNoise_Q;
    Mat topLeftCovariance_P = covariance_P.colRange(0,3).rowRange(0,3);
    predictedPoseCovariance.copyTo(topLeftCovariance_P);

    int covarianceCols = covariance_P.cols;
    if (covarianceCols > 3){
        Mat updatedTop3Rows = stateTransitionJacobian_A * covariance_P(cv::Rect(3, 0, covarianceCols - 3, 3)) ;
        Mat covarianceTop3Rows = covariance_P.colRange(3,covarianceCols).rowRange(0,3);
        updatedTop3Rows.copyTo(covarianceTop3Rows);


        // May not be necessary (Maintain a triangular matrix)
        Mat covarianceFirst3Cols = covariance_P.colRange(0, 3).rowRange(3, covarianceCols) = updatedTop3Rows.t();
    }

}

// --------------------------
// Updation helper functions
// --------------------------
// Update the measurement jacobian H for the current landmark
void EkfSlam::updateMeasurementJacobian(int currLandmarkIdx, float rX, float rY, float lX, float lY, float expectedRange){
    // Jacobian matrix has the form (Only robot pose and current landmark's columns are non-zero)
    //  rX  rY  rT  0  ... l_ri    l_bi ... l_rn    l_bn
    //  ------------------------------------------------
    //  [A  B   C   0   ... -A      -B  ... 0       0]
    //  [D  E   F   0   ... -D      -E  ... 0       0]

    float A = (rX - lX) / expectedRange;
    float B = (rY - lY) / expectedRange;
    float C = 0.0f;
    float D = (lY - rY) / (expectedRange * expectedRange);
    float E = (lX - rX) / (expectedRange * expectedRange);
    float F = -1.0f;

    measurementJacobian_H = Mat::zeros(2, state_x.rows, CV_32F);
    // Set robot pose values
    measurementJacobian_H.at<float>(0,0) = A;
    measurementJacobian_H.at<float>(0,1) = B;
    measurementJacobian_H.at<float>(0,2) = C;
    measurementJacobian_H.at<float>(1,0) = D;
    measurementJacobian_H.at<float>(1,1) = E;
    measurementJacobian_H.at<float>(1,2) = F;
    // Set current landmark values
    measurementJacobian_H.at<float>(0, currLandmarkIdx) = -A;
    measurementJacobian_H.at<float>(0, currLandmarkIdx + 1) = -B;
    measurementJacobian_H.at<float>(1, currLandmarkIdx) = -D;
    measurementJacobian_H.at<float>(1, currLandmarkIdx + 1) = -E;
}

// TODO: Handle when a previously observed landmark is not observed this time.
// Find the closest incoming landmark by euclidean distance and pass it through a validation gate.
bool EkfSlam::associateLandmark(float expectedRange, float expectedTheta, Mat innovationCovariance_S){
    float minDist = INFINITY;
    int bestIdx = -1;
    for (int i = 0; i < (int) m_goodMeasurements.size(); i++){
        float currRange = m_goodMeasurements[i].dist;
        float currTheta = m_goodMeasurements[i].angle;
        float d = sqrt((currRange * currRange) + (expectedRange * expectedRange) - (2 * currRange * expectedRange * cos(currTheta - expectedTheta)));
        if (d < minDist){
            minDist = d;
            bestIdx = i;
        }
    }
    if (minDist < INFINITY){
        float bestRange = m_goodMeasurements[bestIdx].dist;
        float bestTheta = m_goodMeasurements[bestIdx].angle;
        Mat innovation = Mat(2, 1, CV_32F);
        innovation.at<float>(0, 0) = (bestRange - expectedRange);
        innovation.at<float>(1, 0) = (bestTheta - expectedTheta);

        // Mat measurementErrorMahDist = innovation.t() * measurementNoise_R.inv() * innovation;
        Mat mahalanobisDistSqrd = innovation.t() * innovationCovariance_S.inv() * innovation;
        assert(mahalanobisDistSqrd.rows == 1 && mahalanobisDistSqrd.cols == 1);
        float mahalanobisDist = sqrt(mahalanobisDistSqrd.at<float>(0,0));
        // Validation gate
        if (mahalanobisDist <= m_associationGate){
            associatedLandmark_z.at<float>(0,0) = bestRange;
            associatedLandmark_z.at<float>(1,0) = bestTheta;
            swap(m_goodMeasurements[bestIdx], m_goodMeasurements.back());
            m_goodMeasurements.pop_back();
            return true;
        }
        else {
            return false;
        }
    }
    return false;
}


// --------------------------
// Landmark management functions
// --------------------------

// Checking if points closest to a walll are the same may or may not tell us if we are measuring the same wall
// The point closest to the wall changes as the robot moves
// The linse from the robot to the wall are parallel even as the robot moves
// Check if the lines from the robot to the wall are parallel to associate walls and therefore landmarks.
// Store the closest point from the origin to the wall in the state vector. This will be consistent throughout robot motion.
// When we get a new measurement (point closest to wall from robot), find the line corresponding to that point, then find the point closest from the origin to that point
// This will be relatively consistent throughout robot motion

void EkfSlam::manageLandmarks(vector<scanDot> measurements){
    // Extract line out of points
    // Find closest point on that line to global origin
    globalizeLandmarks(measurements);
    loadLandmarks();
    updateLandmarkStatus();
}

// Measurements comes in as the point closest from a wall to the robot.
// Find the point closest from this line to the origin
void EkfSlam::globalizeLandmarks(vector<scanDot> measurements){
    m_globalizedMeasurements.clear();
    float robotX = state_x.at<float>(0,0);
    float robotY = state_x.at<float>(1,0);
    float robotTheta = state_x.at<float>(2,0);

    for (int i = 0; i < (int) measurements.size(); i++){
        float currDist = measurements[i].dist;
        float currAng = measurements[i].angle;
        float globalTheta = currAng + robotTheta;
        // Calculate the slope of the line in global coordinates
        float run = currDist * std::cos(globalTheta);
        float rise = currDist * std::sin(globalTheta);
        // float run = currDist * std::cos(currAng);
        // float rise = currDist * std::sin(currAng);
        float slope = rise / run;

        // Compute a single point in the line in global coordinates
        float globalX = robotX + run;
        float globalY = robotY + rise;
        // float globalX = robotX + (currDist * std::cos(globalTheta));
        // float globalY = robotY + (currDist * std::sin(globalTheta));

        // Compute the line in global coordinates;
        // y = mx + b
        // 0 = mx + b - y
        // 0 = Ax + By + C (A = m, B = -1, C = b)
        float lineA = -1 / slope; // "slope" is the slope of the line from robot to wall. We want the line perpendicular to that.  
        float lineB = -1;
        float lineC = globalY - (lineA * globalX);

        // Find the point closest to the this line from the origin
        float denom = (lineA * lineA) + (lineB * lineB);
        float x = - (lineA * lineC) / denom;
        float y = - (lineB * lineC) / denom;

        // Handle verticle line
        if (fabs(run) < 1.0e-4){
            x = 0;
            y = globalY;
        }
        // Handle horizontal line
        else if (fabs(rise) < 1.0e-4){
            x = globalX;
            y = 0;
        }

        m_globalizedMeasurements.push_back(Point2f(x, y));
    }
}

// Update observed landmarks database by updating re-observed landmarks and creating newly observed landmarks
void EkfSlam::loadLandmarks(){
    // Does this landmark already exist?
    //  - If yes: increase observation count
    //  - If no: Initialize new landmark

    for (int i = 0; i < (int) m_globalizedMeasurements.size(); i++){
        float measuredX = m_globalizedMeasurements[i].x;
        float measuredY = m_globalizedMeasurements[i].y;
        // float bestMahDist = INFINITY; // Mahalanobis Distance
        float bestDist = INFINITY;
        float bestIdx = -1;
        for (int j = 0; j < (int) m_observedLandmarks.size(); j++){
            float expectedX = m_observedLandmarks[j].point.x;
            float expectedY = m_observedLandmarks[j].point.y;
            float xDiff = measuredX - expectedX;
            float yDiff = measuredY - expectedY;

            float dist = sqrt(((xDiff * xDiff) + (yDiff * yDiff)));

            if (dist < bestDist){
                bestDist = dist;
                bestIdx = j;
            }

        }
        if (bestDist <= 300.0f){
            // Found association
            m_observedLandmarks[bestIdx].point.x = measuredX;
            m_observedLandmarks[bestIdx].point.y = measuredY;
            m_observedLandmarks[bestIdx].recentlyObserved = true;

        }
        // No association found. Create a new landmark and add to observedLandmarks
        else {
            // Convert local coords to global
            Landmark newLandmark;
            newLandmark.observationCount = 0;
            newLandmark.recentlyObserved = true;
            newLandmark.status = unconfirmed;
            newLandmark.point.x = measuredX;
            newLandmark.point.y = measuredY;
            m_observedLandmarks.push_back(newLandmark);

        }
    }
}

void EkfSlam::updateLandmarkStatus(){
    m_goodMeasurements.clear();
    float robotX = state_x.at<float>(0,0);
    float robotY = state_x.at<float>(1,0);
    float robotTheta = state_x.at<float>(2,0);

    for (int i = 0; i < (int) m_observedLandmarks.size(); i++){
        if (!m_observedLandmarks[i].recentlyObserved && m_observedLandmarks[i].status == unconfirmed){
            m_observedLandmarks[i].observationCount -= 2;
            if (m_observedLandmarks[i].observationCount <= 0){
                // Remove this landmark 
                swap(m_observedLandmarks[i], m_observedLandmarks.back());
                m_observedLandmarks.pop_back();
            }
        }
        else if (m_observedLandmarks[i].recentlyObserved) {
            // Convert global coords to local
            m_observedLandmarks[i].observationCount ++;
            m_observedLandmarks[i].recentlyObserved = false;
            if (m_observedLandmarks[i].observationCount >= m_landmarkConfirmationCount){ 
                if (m_observedLandmarks[i].status == unconfirmed){
                    m_observedLandmarks[i].status = confirmed;
                }
                // TODO: Can directly push global cartesian coord and avoid reconverting later
                scanDot goodLandmark;
                float globalX = m_observedLandmarks[i].point.x;
                float globalY = m_observedLandmarks[i].point.y;
                float xDiff = globalX - robotX;
                float yDiff = globalY - robotY;
                goodLandmark.angle = atan2(yDiff, xDiff) - robotTheta;
                goodLandmark.dist = sqrt((xDiff * xDiff) + (yDiff * yDiff));
            
                m_goodMeasurements.push_back(goodLandmark);
            }
        }
    }
}

#if DISPLAY_LANDMARKS
void EkfSlam::displayLandmarks(){
    Mat image = Mat::zeros(800, 800, CV_8UC3);
    Point center = Point(image.rows / 2, image.cols / 2);
    Scalar green = Scalar(0, 255, 0);
    Scalar red = Scalar(0,0,255);
    Scalar blue = Scalar(255,0,0);
    circle(image, center, 5, green);

    // int scaleFactor = 20;
    for (int i = 0; i < (int) m_observedLandmarks.size(); i++){
        if (m_observedLandmarks[i].status != confirmed){
            continue;
        }
        Point p = Point(m_observedLandmarks[i].point.x, m_observedLandmarks[i].point.y);
        // circle(image, Point(center.x + (p.x / DISPLAY_SCALE) , center.y - (p.y / DISPLAY_SCALE)), 3, red, 2);
    }

    for (int i = 3; i < (int) state_x.rows; i += 2){
        
        Point p = Point(state_x.at<float>(i,0), state_x.at<float>(i + 1,0));
        circle(image, Point(center.x + (p.x / DISPLAY_SCALE) , center.y - (p.y / DISPLAY_SCALE)), 3, blue, 2);
    }

    imshow("Observed Landmarks", image);
    waitKey(1);
    return;
}
#endif