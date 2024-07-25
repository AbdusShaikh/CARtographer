#pragma once
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <vector>

using namespace std;

// Single Lidar point struct
struct scanDot {
    uint8_t quality;
    float angle;
    float dist;
};

// Container for wheel encoder readings
struct WheelEncoderDataContainer {
    int leftWheel = 0;
    int leftWheelPrev = 0;
    int rightWheel = 0;
    int rightWheelPrev = 0;
};

// Sensor readings object to be filled by lidar and wheel encoders, and consumed by mastermind
struct Observation {
    vector<scanDot> lidarReadings;
    WheelEncoderDataContainer wheelEncoderReadings;
};