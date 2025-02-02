#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <cmath>
#include <vector>
#include "autonomous_navigation/pid_controller.hpp"

struct Point
{
    double x; // [m]
    double y; // [m]

    Point(double x, double y) : x(x), y(y) {}

    bool operator==(const Point &point) const;
};

struct Pose
{
    Point position;
    double orientation; // yaw angle [rad]

    Pose(Point position, double orientation) : position(position), orientation(orientation) {}
};

class PurePursuitController
{
public:
    /**
     * @brief Constructor for the PurePursuitController.
     *
     * Initializes the controller with a path and parameters for adjusting the motion control.
     *
     * @param path The list of points that define the path the vehicle needs to follow.
     * @param v_max The maximum velocity the vehicle can travel at.
     * @param ld_min The minimum lookahead distance.
     * @param K_v_turn A constant used to adjust the exponential deceleration during turns, which depends on the steering angle.
     * @param K_v_stop A constant used to adjust the proportional gain of the P controller that regulates deceleration when approaching the final path point.
     * @param K_ld A constant used to adjust the lookahead distance, which is proportional to the current linear velocity of the vehicle.
     */
    PurePursuitController(const std::vector<Point> &path = {}, double v_max = 1.0, double ld_min = 0.15, double K_v_turn = 1.0, double K_v_stop = 1.0, double K_ld = 0.5);

    /**
     * @brief Computes the steering angle and velocity to drive the vehicle along the path using the Pure Pursuit algorithm.
     *
     * This method computes the steering angle and the angular velocity of the rear wheels for the vehicle to follow the desired path using the Pure Pursuit algorithm,
     * based on the kinematic bicycle model of the 4-wheel vehicle.
     *
     * @param current_pose The current pose of the vehicle's rear wheel axle, which includes its position and orientation.
     * @param current_velocity The current average angular velocity (in radians per second) of the rear wheels.
     * @return A pair containing the computed steering angle (in radians) and the angular velocity for the rear wheels (in radians per second).
     * @throws std::runtime_error If the target point is behind the vehicle or if the computed steering angle exceeds the vehicle's steering limits.
     */
    std::pair<double, double> get_motion_controls(Pose current_pose, double current_velocity);
    void set_path(const std::vector<Point> &path);

private:
    Point find_target_point(Point current_position, double lookahead_distance);
    int find_closest_path_point_index(Point current_position, size_t start_index);
    bool is_point_on_line_segment(Point p, Point p1, Point p2);
    std::optional<std::vector<Point>> find_line_segment_circle_intersections(Point line_point_1, Point line_point_2, Point circle_center, double circle_radius);
    Point transform_to_local_frame(Point global_point, Pose vehicle_pose);

    std::vector<Point> path_;
    size_t last_closest_path_point_index_ = 0;
    double v_max_;
    double ld_min_;
    double K_v_turn_;
    double K_v_stop_;
    double K_ld_;
    PIDController v_stop_controller_;
};

#endif // PURE_PURSUIT_HPP
