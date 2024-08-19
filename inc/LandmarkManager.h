#include "common.h"
// Purpose:
//  - Centralized store of "good" landmarks
//      - Landmarks that have appeared enough times to be considered real
// Operations
//  - Consume measurements vector and categorize landmarks based on observation count
//  - Get list of good landmarks
//      - Will be processed in update step of EKF
//  - Associate landmark indices in EKF state vector to a good measurement for that landmark for easy access


enum landmarkStatus{unconfirmed = 0, confirmed = 1};

struct Landmark {
    scanDot point;
    int observationCount;
    landmarkStatus status;
    bool recentlyObserved;
};

class LandmarkManager {
    public:
        LandmarkManager();
        ~LandmarkManager();
        vector<scanDot> step(vector<scanDot> measurements, float distTravelled);
        void load(vector<scanDot> measurements, float distTravelled);
        void updateStatuses();

    private:
        vector<Landmark> observedLandmarks;
        vector<scanDot> goodLandmarks;

        int confirmationCount = 30;
};