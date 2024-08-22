#include <opencv2/core.hpp>
#include "common.h"
#define DISPLAY_LANDMARKS 1

#if DISPLAY_LANDMARKS
    #include <opencv2/opencv.hpp>
    #include <opencv2/highgui.hpp>
    #include <opencv2/imgproc.hpp>
#endif

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
        bool associateLandmark(float expectedRange, float expectedTheta, Mat innovationCovariance);
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
        int m_landmarkConfirmationCount;
        float m_landmarkMaxDist;
};