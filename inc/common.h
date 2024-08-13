#pragma once
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <vector>
#include <assert.h>

using namespace std;

// Single Lidar point struct
struct scanDot {
    float angle;
    float dist;
};

struct OdometryDataContainer {
    float leftWheelDist = 0.0f; // mm
    float rightWheelDist = 0.0f; // mm
    float carWheelBase = 0.0f; // mm
    // TODO:
    // --- May not need these ---
    float dx = 0.0f; // mm
    float dy = 0.0f; // mm
    float dTheta = 0.0f; // Radians
    float globalX = 0.0f; // mm
    float globalY = 0.0f; // mm
    float globalTheta = 0.0f; // mm
};

// Sensor readings object to be filled by lidar and wheel encoders, and consumed by mastermind
struct Observation {
    // vector<scanDot> lidarReadings;
    vector<scanDot> lidarExtractedLandmarks;
    OdometryDataContainer odometry;
};