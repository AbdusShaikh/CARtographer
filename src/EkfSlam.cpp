#include "EkfSlam.h"
#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif
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
//  - measurementNoise_R is different in addNewLandmark and update step

EkfSlam::EkfSlam(){};
EkfSlam::~EkfSlam(){};

int EkfSlam::init(){
    state_x = VectorXf(3).setZero(); // Initial robot pose is at "origin"
    covariance_P = MatrixXf(3,3).setZero();
    measurementNoise_R.setZero();
    identity_V.setIdentity();
    associatedLandmark_z.setZero();
    stateTransitionJacobian_A = Matrix3f().setIdentity();

    m_associationGate = 4.6f; // Based off Chi-Square distribution
    m_RangeVariance = 8.5f;
    m_BearingVariance = 1.0f; // 1 radian

    m_odometryError = 0.5f;
    m_dxVariance = 0.1f;
    m_dyVariance = 5.15;
    m_dThetaVariance = 0.02;

    m_landmarkConfirmationCount = 50;
    m_landmarkMaxDist = 300.0f;

    return EXIT_SUCCESS;
}

// ------------------------
// Main Algorithm functions
// ------------------------

// "Main" function of Kalman Filter. Executes predict and update steps
VectorXf EkfSlam::step(vector<scanDot> measurements, OdometryDataContainer controlInputs){
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

    float currX = state_x(0,0);
    float currY = state_x(1,0);
    float currTheta = state_x(2,0);

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
    for (int i = 3; i < state_x.rows(); i += 2){ // Landmarks start at row 3
        // These values change as we update the state vector so we need to reinitialize them each tiem
        float rX = state_x(0 ,0);
        float rY = state_x(1, 0);
        float rTheta = state_x(2, 0);

        // 1.) Calculate estimated position of landmark
        float lX = state_x(i, 0); // Current landmark's x position in world coordinates
        float lY = state_x(i + 1, 0); // landmark's y position in world coordinates
        // Measurement model
        float xDiff = lX - rX;
        float yDiff = lY - rY;
        float expectedRange = sqrt((xDiff * xDiff) + (yDiff * yDiff)); // Expected landmark range based on the predicted pose
        float expectedBearing = atan2(yDiff, xDiff) - rTheta; // Expected landmark bearing based on the predicted pose

        Vector2f predictedLandmark_h;
        predictedLandmark_h(0, 0) = expectedRange;
        predictedLandmark_h(1, 0) = expectedBearing;

        // 2.) Create measurement model Jacobian H
        updateMeasurementJacobian(i, rX, rY, lX, lY, expectedRange);

        // 3.) Update measurement noise matrix R
        measurementNoise_R(0, 0) = m_RangeVariance * expectedRange;
        measurementNoise_R(1, 1) = m_BearingVariance;


        // 4.) Compute innovation covariance S
        Matrix2f innovationCovariance_S = (measurmentJacobian_H * covariance_P * measurmentJacobian_H.transpose()) + (identity_V * measurementNoise_R * identity_V.transpose());
        
        // 5.) Find associated landmark
        if (!associateLandmark(expectedRange, expectedBearing, innovationCovariance_S)){ // Failed to find this previously observed landmark in current reading. We want to only update reobserved landmarks
            continue;
        }
        // 6.) Compute Kalman Gain
        kalmanGain_K = covariance_P * measurmentJacobian_H.transpose() * innovationCovariance_S.inverse();

        // 7.) Update state        
        state_x = state_x + (kalmanGain_K * (associatedLandmark_z - predictedLandmark_h));

        // 8.) Update covariance
        covariance_P = covariance_P - (kalmanGain_K * innovationCovariance_S * kalmanGain_K.transpose());

    }
};


// Dynamicallly grow state vector (AKA The Map) based on observations
void EkfSlam::addNewLandmarks(){
    float robotX = state_x(0, 0);
    float robotY = state_x(1, 0);
    float robotTheta = state_x(2, 0);

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
        // TODO: No need to remake the whole matrix each time
        MatrixXf invMeasurementPoseJacobian = MatrixXf(2, 3).setIdentity();
        invMeasurementPoseJacobian(0, 2) = -landmarkR * sinGlobalTheta;
        invMeasurementPoseJacobian(1, 2) = landmarkR * cosGlobalTheta;

        // Jacobian matrix of the inverted measurement model with respect to range and bearing of new landmark
        Matrix2f invMeasurementLandmarkJacobian;
        invMeasurementLandmarkJacobian(0, 0) = cosGlobalTheta;
        invMeasurementLandmarkJacobian(0, 1) = -landmarkR * sinGlobalTheta;;
        invMeasurementLandmarkJacobian(1, 0) = sinGlobalTheta;
        invMeasurementLandmarkJacobian(1, 1) = landmarkR * cosGlobalTheta;

        measurementNoise_R(0, 0) = m_RangeVariance;
        measurementNoise_R(1, 1) = m_BearingVariance;

        // Calculate new covariance submatrix and crossvariance vectors
        Matrix2f newLandmarkCovariance = (invMeasurementPoseJacobian * covariance_P.block(0, 0, 3, 3) * invMeasurementPoseJacobian.transpose()) + (invMeasurementLandmarkJacobian * measurementNoise_R * invMeasurementLandmarkJacobian.transpose());
        MatrixXf newLandmarkCrossVariance = invMeasurementPoseJacobian * covariance_P.block(0, 0, 3, covariance_P.cols());

        int prevRows = covariance_P.rows(), prevCols = covariance_P.cols();
        // Add two new rows and columns to the covariance matrix
        covariance_P.conservativeResize(prevRows + 2, prevCols + 2);
        // Add the landmark cross variances to the new rows and columns to the covariance matrix
        covariance_P.block(prevRows, 0, 2, prevCols) = newLandmarkCrossVariance;
        covariance_P.block(0, prevCols, prevRows, 2) = newLandmarkCrossVariance.transpose();
        // Add the covariance for this new landmark to the covariance matrix
        covariance_P.block(prevRows, prevCols, 2, 2) = newLandmarkCovariance;

        // Add this new landmark to the state vector
        state_x.conservativeResize(state_x.size() + 2);
        state_x(state_x.size() - 2) = landmarkGlobalX;
        state_x(state_x.size() - 1) = landmarkGlobalY;
        

    }
}
// --------------------------
// Prediction helper functions
// --------------------------

// Predicts where the landmarks in the previous state will appear now in robot frame
void EkfSlam::updateTransitionJacobian(float dx_mm, float dy_mm){
    stateTransitionJacobian_A(0, 2) = -dy_mm;
    stateTransitionJacobian_A(1, 2) = dx_mm;
}

void EkfSlam::predictStateVec(float predictedX, float predictedY, float predictedTheta){
    state_x(0,0) = predictedX;
    state_x(1,0) = predictedY;
    state_x(2,0) = predictedTheta;

}

//TODO: Find proper noise measurements
void EkfSlam::updateProcessNoise(float dx_mm, float dy_mm, float dTheta){
    Vector3f W =  Vector3f().setZero();
    W(0,0) = dx_mm;
    W(1,0) = dy_mm;
    W(2,0) = dTheta;
    processNoise_Q = W * m_odometryError * W.transpose();
    
}

// Update robot pose covariance and robot to feature correlations
void EkfSlam::predictCovarianceMat(){
    // Update robot pose covariance
    Matrix3f predictedPoseCovaraince = (stateTransitionJacobian_A * covariance_P.block(0, 0, 3, 3) * stateTransitionJacobian_A.transpose()) + processNoise_Q;
    covariance_P.block(0,0,3,3) = predictedPoseCovaraince;

    int covarianceCols = covariance_P.cols();
    if (covarianceCols > 3){
        // Update robot-feature cross-variance
        MatrixXf updatedTop3Rows = stateTransitionJacobian_A * covariance_P.block(0, 3, 3, covarianceCols - 3);
        covariance_P.block(0, 3, 3, covarianceCols - 3) = updatedTop3Rows;

        // Update feature-update cross-variance
        covariance_P.block(3, 0, covarianceCols - 3, 3) = updatedTop3Rows.transpose();
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

    measurmentJacobian_H = MatrixXf(2, state_x.rows()).setZero();
    // Set robot pose values
    measurmentJacobian_H(0,0) = A;
    measurmentJacobian_H(0,1) = B;
    measurmentJacobian_H(0,2) = C;
    measurmentJacobian_H(1,0) = D;
    measurmentJacobian_H(1,1) = E;
    measurmentJacobian_H(1,2) = F;
    // Set current landmark values
    measurmentJacobian_H(0, currLandmarkIdx) = -A;
    measurmentJacobian_H(0, currLandmarkIdx + 1) = -B;
    measurmentJacobian_H(1, currLandmarkIdx) = -D;
    measurmentJacobian_H(1, currLandmarkIdx + 1) = -E;

}

// TODO: Handle when a previously observed landmark is not observed this time.
// Find the closest incoming landmark by euclidean distance and pass it through a validation gate.
bool EkfSlam::associateLandmark(float expectedRange, float expectedTheta, const Matrix2f innovationCovariance_S){
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
        Vector2f innovation;
        innovation(0, 0) = (bestRange - expectedRange);
        innovation(1, 0) = (bestTheta - expectedTheta);
        float mahalanobisDist = sqrt(innovation.transpose() * innovationCovariance_S.inverse() * innovation);

        // Validation gate
        if (mahalanobisDist <= m_associationGate){
            associatedLandmark_z(0,0) = bestRange;
            associatedLandmark_z(1,0) = bestTheta;

            std::swap(m_goodMeasurements[bestIdx], m_goodMeasurements.back());
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
// Measurements vector comes in  as the point on the wall that is closest to the robot, in robot frame
// In order to effectively predict and track a wall, we must convert the point on wall W that is closest to the robot, to the point that is closest to the origin on the same wall W
// This point will be relatively consisten throughout robot motion, and thus enable effective data association, and in turn localization and mapping

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
    float robotX = state_x(0,0);
    float robotY = state_x(1,0);
    float robotTheta = state_x(2,0);
    

    for (int i = 0; i < (int) measurements.size(); i++){
        float currDist = measurements[i].dist;
        float currAng = measurements[i].angle;
        float globalTheta = currAng + robotTheta;
        // Calculate the slope of the line in global coordinates
        float run = currDist * std::cos(globalTheta);
        float rise = currDist * std::sin(globalTheta);
        float slope = rise / run;

        // Compute a single point in the line in global coordinates
        float globalX = robotX + run;
        float globalY = robotY + rise;

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
        if (bestDist <= m_landmarkMaxDist){
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
    float robotX = state_x(0, 0);
    float robotY = state_x(1, 0);
    float robotTheta = state_x(2, 0);
    for (int i = 0; i < (int) m_observedLandmarks.size(); i++){
        if (!m_observedLandmarks[i].recentlyObserved && m_observedLandmarks[i].status == unconfirmed){
            m_observedLandmarks[i].observationCount -= 2;
            if (m_observedLandmarks[i].observationCount <= 0){
                // Remove this landmark 
                std::swap(m_observedLandmarks[i], m_observedLandmarks.back());
                m_observedLandmarks.pop_back();
            }
        }
        else if (m_observedLandmarks[i].recentlyObserved) {
            // Convert global coords to local
            m_observedLandmarks[i].observationCount ++;
            m_observedLandmarks[i].recentlyObserved = false;
            if (m_observedLandmarks[i].observationCount >= m_landmarkConfirmationCount){ 
                m_observedLandmarks[i].status = confirmed;
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
    cv::Point center = cv::Point(image.rows / 2, image.cols / 2);
    cv::Scalar green = cv::Scalar(0, 255, 0);
    cv::Scalar red = cv::Scalar(0,0,255);
    cv::Scalar blue = cv::Scalar(255,0,0);
    circle(image, center, 5, green);

    // int scaleFactor = 20;
    // for (int i = 0; i < (int) m_observedLandmarks.size(); i++){
    //     if (m_observedLandmarks[i].status != confirmed){
    //         continue;
    //     }
    //     Point p = Point(m_observedLandmarks[i].point.x, m_observedLandmarks[i].point.y);
    //     circle(image, Point(center.x + (p.x / DISPLAY_SCALE) , center.y - (p.y / DISPLAY_SCALE)), 3, red, 2);
    // }

    for (int i = 3; i < (int) state_x.rows(); i += 2){
        
        Point p = Point(state_x(i,0), state_x(i + 1,0));
        Matrix2f covSubmatrix = covariance_P.block(i, i, 2, 2);
        EigenSolver<MatrixXf> eigenSolver(covSubmatrix);
        Vector2f eigenValues = eigenSolver.eigenvalues().real();
        Matrix2f eigenVectors = eigenSolver.eigenvectors().real();
        float ev1 = eigenValues[0];
        float ev2 = eigenValues[1];

        Vector2f eigenvector1 = eigenVectors.col(0);
        Vector2f eigenvector2 = eigenVectors.col(1);

        float angle, majorAxis, minorAxis;
        if (ev1 > ev2){
            angle = atan2(eigenvector1(1), eigenvector1(0)) * (180.0f / M_PI);
            majorAxis = std::sqrt(ev1);
            minorAxis = std::sqrt(ev2);
        }
        else {
            angle = atan2(eigenvector2(1), eigenvector2(0))  * (180.0f / M_PI);
            majorAxis = std::sqrt(ev2);
            minorAxis = std::sqrt(ev1);
        }

        // Draw the ellipse
        cv::ellipse(image, Point(center.x + (p.x / DISPLAY_SCALE) , center.y - (p.y / DISPLAY_SCALE)), cv::Size(majorAxis, minorAxis), angle, 0, 360, green, 2);


        circle(image, Point(center.x + (p.x / DISPLAY_SCALE) , center.y - (p.y / DISPLAY_SCALE)), 2, red, 2);
    }

    imshow("Observed Landmarks", image);
    waitKey(1);
    return;
}
#endif