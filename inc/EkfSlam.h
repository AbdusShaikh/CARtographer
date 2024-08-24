#include <opencv2/core.hpp>
#include "common.h"
#include <eigen3/Eigen/Eigen>
#define DISPLAY_LANDMARKS 1

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
        bool associateLandmark(float expectedRange, float expectedTheta, const Mat innovationCovariance_S, const Matrix2f innovationCovariance_S_Eigen);
        // Utility functions
        void displayLandmarks();
        // Feature/Landmark Management
        void manageLandmarks(vector<scanDot> measurements);
        void globalizeLandmarks(vector<scanDot> measurements);
        void loadLandmarks();
        void updateLandmarkStatus();

        // Matrices
        Mat stateTransitionJacobian_A;
        Mat covariance_P;
        Mat processNoise_Q;
        Mat measurementNoise_R;
        Mat measurementJacobian_H;
        // Mat innovationCovariance_S;
        Mat kalmanGain_K;
        Mat identity_V;

        // Vectors
        // State vectors (Robot pose and landmark position)
        Mat state_x;
        Mat associatedLandmark_z;

        // --------
        // Eigen version

        // Matrices
        MatrixXf stateTransitionJacobian_A_Eigen;
        MatrixXf covariance_P_Eigen;
        Matrix3f processNoise_Q_Eigen;
        Matrix2f measurementNoise_R_Eigen;
        MatrixXf measurmentJacobian_H_Eigen;
        MatrixXf kalmanGain_K_Eigen;
        Matrix2f identity_V_Eigen;

        // Vectors
        // State vectors (Robot pose and landmark position)
        VectorXf state_x_Eigen;
        Vector2f associatedLandmark_z_Eigen;

        // Scalar values
        float m_odometryError;
        float m_RangeVariance;
        float m_BearingVariance;
        float m_associationGate;

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
        vector<Point2f> m_globalizedMeasurements_Eigen;

        int m_landmarkConfirmationCount;
        float m_landmarkMaxDist;
};