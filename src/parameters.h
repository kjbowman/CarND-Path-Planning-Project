#ifndef PARAMETERS_H
#define PARAMETERS_H

// simulator/model parameters
constexpr unsigned N_LANES = 3;
constexpr unsigned LEFT_LANE = 0;           // midside lane
constexpr unsigned RIGHT_LANE = N_LANES-1;  // outside lane
constexpr double LANE_WIDTH = 4.0;          // [m]
constexpr double SIMULATOR_CYCLE_TIME = 0.02;    // [s] (20 ms)
constexpr double UPDATE_RATE = 1 / SIMULATOR_CYCLE_TIME;    // [Hz] (20 ms => 50 Hz)

// conversion constants
constexpr double METERS_PER_MILE = 1609.3;
constexpr double SECONDS_PER_HOUR = 3600;

// project constraints
constexpr double ROAD_SPEED_LIMIT = 50.0;
constexpr double MAX_ACCEL = 10.0;          // [m/s^2]
constexpr double MAX_JERK = 10.0;           // [m/s^3]
constexpr double MAX_LANE_CHANGE_TIME = 3.0;  // [s]

// constants for tuning the algorithm
constexpr int PATH_BUFFER_SIZE = 30;  // 50
constexpr int NUM_SPLINE_PTS = 4;
constexpr double SPLINE_PTS_SPACING = 30.0;   // [m]
constexpr double CAR_LENGTH = 5.0;            // [m] (a guess based on avg car length)
constexpr double FOLLOWING_GAP = CAR_LENGTH * 3;        // [m]
constexpr double CUTIN_GAP = CAR_LENGTH * 2;            // [m]
constexpr double SPEED_LIMIT = ROAD_SPEED_LIMIT - 0.5;  // [mph]
constexpr double SPEED_INCREMENT = 0.25;      // [mph/update_rate]

// Sensor fusion list indexes
constexpr int SF_ID = 0;
constexpr int SF_X = 1;
constexpr int SF_Y = 2;
constexpr int SF_VX = 3;
constexpr int SF_VY = 4;
constexpr int SF_S = 5;
constexpr int SF_D = 6;

#endif  // PARAMETERS_H