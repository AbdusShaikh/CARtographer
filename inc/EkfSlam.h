#include <opencv2/core/mat.hpp>
#include "common.h"

class EkfSlam {
    public:
        EkfSlam();
        ~EkfSlam();
        int init();
        cv::Mat step(vector<scanDot> measurements, OdometryDataContainer controlInputs);
    private:
        // Algorithm functions
        void predict();
        void update();
        // Prediction step helper-functions
        void predictMeasurements();
        void predictStateVec(float predictedX, float predictedY, float predictedTheta);
        void updateTransitionJacobian(float dx_mm, float dy_mm);
        void updateProcessNoise(float dx_mm, float dy_mm, float dTheta);
        void predictCovarianceMat();
        // Update step helpfer functions
        // Utility functions
        void createInputsVec(OdometryDataContainer controlInputs);
        void createMeasurementsVec(vector<scanDot> measurements);
        void robotToWorldCoord(float* worldR, float* worldTheta, float robotR, float robotTheta);
        void worldToRobotCoord(float* robotR, float* robotTheta, float worldR, float worldTheta);

        // Matrices
        // cv::Mat stateTransition_F;
        cv::Mat stateTransitionJacobian_A;
        // cv::Mat control_G;
        cv::Mat predictedCovariance_P;
        cv::Mat trueCovariance_P;
        cv::Mat processNoise_Q;
        cv::Mat measurementNoise_R;
        cv::Mat observation_H;
        cv::Mat kalmanGain_K;

        // Vectors
        // State vectors (Robot pose and landmark position)
        cv::Mat predictedState_x;
        cv::Mat trueState_x;
        // Output of Lidar reading
        cv::Mat measurement_z;
        cv::Mat predictedMeasurement_z;
        // (dx, dy, dTheta)
        cv::Mat input_u;
        // cv::Mat processNoise_w;
        // cv::Mat measurementNoise_v;

        // Scalar values
        float m_odometryError = 0.0f;
        float m_measurementNoiseRange = 0.0f;
        float m_measurementNoiseBearing = 0.0f;

        // Extermanl input
        OdometryDataContainer m_controlInputs;
        vector<scanDot> m_measurements; 
};