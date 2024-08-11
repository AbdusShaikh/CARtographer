#include <opencv2/core/mat.hpp>
#include "common.h"

class EkfSlam {
    public:
        EkfSlam();
        ~EkfSlam();
        int init();
        cv::Mat step(vector<scanDot> measurements, OdometryDataContainer controlInputs);
        cv::Mat getState();
    private:
        // Algorithm functions
        void predict();
        void update();
        // Algorithm helper-functions
        void predictMeasurements();
        void associateMeasurements(float distThreshold); // Nearest-neighbour based landmark data association algorithm. Rewrite the current measurements Matrix to match the order of the state vector.
        // Utility functions
        void createInputsVec(OdometryDataContainer controlInputs);
        void createMeasurementsVec(vector<scanDot> measurements);
        void createObservationJacobian();
        void robotToWorldCoord(float* worldR, float* worldTheta, float robotR, float robotTheta);
        void worldToRobotCoord(float* robotR, float* robotTheta, float worldR, float worldTheta);

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
        cv::Mat predictedMeasurement_z;
        // (dx, dy, dTheta)
        cv::Mat input_u;
        cv::Mat processNoise_w;
        cv::Mat measurementNoise_v;
};