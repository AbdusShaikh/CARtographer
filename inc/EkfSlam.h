#include <opencv2/core/mat.hpp>
#include "common.h"

class EkfSlam {
    public:
        EkfSlam();
        ~EkfSlam();
        int init();
        cv::Mat step(vector<vector<scanDot>> measurements);
        cv::Mat getState();
    private:
        void predict();
        void update();
        void createMeasurementsMat(vector<vector<scanDot>> measurements);
        void padInput();
        cv::Mat predictMeasurements();
        // Matrices
        cv::Mat stateTransition_F;
        cv::Mat control_G;
        cv::Mat predictedEstimateCovariance_P;
        cv::Mat trueEstimateCovariance_P;
        cv::Mat processNoiseCovariance_Q;
        cv::Mat measurementCovariance_R;
        cv::Mat observation_H;
        cv::Mat kalmanGain_K;

        // Vectors
        // State vectors (Robot pose and landmark position)
        cv::Mat predictedState_x;
        cv::Mat trueState_x;
        // Output of Lidar reading
        cv::Mat measurement_z;
        // (dx, dy, dTheta)
        cv::Mat input_u;
        cv::Mat processNoise_w;
        cv::Mat measurementNoise_v;
};