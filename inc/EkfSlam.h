#include <opencv2/core.hpp>
#include "common.h"
#include "LandmarkManager.h"

using namespace cv;

class EkfSlam {
    public:
        EkfSlam();
        ~EkfSlam();
        int init();
        Mat step(vector<scanDot> measurements, OdometryDataContainer controlInputs);
    private:
        // Algorithm functions
        void predict();
        void update();
        void addNewLandmarks();
        // Prediction step helper-functions
        void predictStateVec(float predictedX, float predictedY, float predictedTheta);
        void updateTransitionJacobian(float dx_mm, float dy_mm);
        void updateProcessNoise(float dx_mm, float dy_mm, float dTheta);
        void predictCovarianceMat();
        // Update step helpfer functions
        void updateMeasurementJacobian(int currLandmarkIdx, float rX, float rY, float lX, float lY, float expectedRange);
        bool associateLandmark(float expectedRange, float expectedTheta);
        // Utility functions
        // void robotToWorldCoord(float* worldR, float* worldTheta, float robotR, float robotTheta);
        // void worldToRobotCoord(float* robotR, float* robotTheta, float worldR, float worldTheta);

        // Matrices
        Mat stateTransitionJacobian_A;
        Mat covariance_P;
        Mat processNoise_Q;
        Mat measurementNoise_R;
        Mat measurementJacobian_H;
        Mat innovationCovariance_S;
        Mat kalmanGain_K;
        Mat identity_V;

        // Vectors
        // State vectors (Robot pose and landmark position)
        Mat state_x;
        Mat associatedLandmark_z;


        // Scalar values
        float m_odometryError;
        float m_RangeVariance;
        float m_BearingVariance;
        float m_associationGate;

        // Extermanl input
        OdometryDataContainer m_controlInputs;
        vector<scanDot> m_rawMeasurements;
        vector<scanDot> m_goodMeasurements; 

        LandmarkManager m_landmarkManager;
};