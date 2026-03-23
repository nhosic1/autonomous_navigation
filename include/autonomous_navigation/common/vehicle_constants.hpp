#ifndef VEHICLE_CONSTANTS_HPP
#define VEHICLE_CONSTANTS_HPP

constexpr double VEHICLE_WIDTH = 0.62; // [m]
constexpr double VEHICLE_LENGTH = 0.7; // [m]
constexpr double VEHICLE_HEIGHT = 0.37; // [m]
constexpr double CHASSIS_HEIGHT = 0.1; // [m]
constexpr double WHEEL_BASE = 0.6; // [m]
constexpr double WHEEL_RADIUS = 0.1; // [m]
constexpr double WHEEL_SEPARATION = 0.52; // [m]
constexpr double STEREO_CAMERA_HEIGHT = 0.35; // [m]
constexpr double STEREO_CAMERA_BASELINE = 0.18; // [m]
constexpr double STEREO_CAMERA_OFFSET_X = 0.35; // [m]

// Bicycle-model minimum turning radius derived from max inner wheel steering angle:
// R_bicycle = WHEEL_BASE / tan(phi_max_kingpin) + KINGPIN_WIDTH / 2
constexpr double MINIMUM_TURNING_RADIUS = 1.097; // [m]
constexpr double KINGPIN_WIDTH = 0.44; // [m]

#endif // VEHICLE_CONSTANTS_HPP