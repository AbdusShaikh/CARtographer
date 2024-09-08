#include <opencv2/core.hpp>
#include "common.h"
#include <eigen3/Eigen/Eigen>
#define DISPLAY_LANDMARKS 0

#if DISPLAY_LANDMARKS
    #include <opencv2/opencv.hpp>
    #include <opencv2/highgui.hpp>
    #include <opencv2/imgproc.hpp>
#endif

using namespace Eigen;
using namespace cv;


enum landmarkStatus{unconfirmed = 0, confirmed = 1};

struct Landmark {
    // scanDot point;
    Point2f point;
    int observationCount;
    landmarkStatus status;
    bool recentlyObserved;
};


class EkfSlam {
    public:
        EkfSlam();
        ~EkfSlam();
        int init();
        VectorXf step(const vector<scanDot> measurements, const OdometryDataContainer controlInputs, int lidarScanResult);
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
        bool associateLandmark(float expectedRange, float expectedTheta, const Matrix2f innovationCovariance_S_Eigen);
        // Utility functions
        void displayLandmarks();
        // Feature/Landmark Management
        void manageLandmarks(const vector<scanDot> measurements);
        void globalizeLandmarks(const vector<scanDot> measurements);
        void loadLandmarks();
        void updateLandmarkStatus();

        // Main Matrices
        MatrixXf stateTransitionJacobian_A;
        MatrixXf covariance_P;
        Matrix3f processNoise_Q;
        Matrix2f measurementNoise_R;
        MatrixXf measurmentJacobian_H;
        MatrixXf kalmanGain_K;
        Matrix2f identity_V;

        // Vectors
        // State vectors (Robot pose and landmark position)
        VectorXf state_x;
        Vector2f associatedLandmark_z;

        // Extermanl input
        OdometryDataContainer m_controlInputs;
        vector<scanDot> m_rawMeasurements;

        // Feature/Landmark Management
        // Local Coords (Remade each iteration)
        vector<scanDot> m_goodMeasurements;
        // Global Coords (Maintained through iterations)
        vector<Landmark> m_observedLandmarks;
        // Global Coord (Remade each iteration)
        vector<Point2f> m_globalizedMeasurements;
        
        // Scalar values
        float m_odometryError;
        float m_RangeVariance;
        float m_BearingVariance;
        float m_associationGate;
        float m_dxVariance;
        float m_dyVariance;
        float m_dThetaVariance;
        int m_landmarkConfirmationCount;
        float m_landmarkMaxDist;
};