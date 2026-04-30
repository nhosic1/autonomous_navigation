#ifndef VEHICLE_CONSTANTS_HPP
#define VEHICLE_CONSTANTS_HPP

constexpr double VEHICLE_WIDTH = 0.428866; // [m]
constexpr double VEHICLE_LENGTH = 0.911599; // [m]
constexpr double VEHICLE_HEIGHT = 0.36845; // [m]
constexpr double CHASSIS_HEIGHT = 0.0465108; // [m]
constexpr double WHEEL_BASE = 0.56118; // [m]
constexpr double WHEEL_RADIUS = 0.085; // [m]
constexpr double WHEEL_SEPARATION = 0.37; // [m]
constexpr double STEREO_CAMERA_HEIGHT = 0.3428; // [m]
constexpr double STEREO_CAMERA_BASELINE = 0.18; // [m]
constexpr double STEREO_CAMERA_OFFSET_X = 0.23879; // [m]

// Bicycle-model minimum turning radius derived from max inner wheel steering angle:
// R_bicycle = WHEEL_BASE / tan(phi_max_kingpin) + KINGPIN_WIDTH / 2
constexpr double MINIMUM_TURNING_RADIUS = 0.985; // [m]
constexpr double KINGPIN_WIDTH = 0.33; // [m]

#endif // VEHICLE_CONSTANTS_HPP
