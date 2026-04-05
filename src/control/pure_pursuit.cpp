#include <stdexcept>
#include <optional>
#include "autonomous_navigation/common/vehicle_constants.hpp"
#include "autonomous_navigation/control/pure_pursuit.hpp"
#include <algorithm>
#include <sstream>

bool Point::operator==(const Point &point) const
{
    const double epsilon = 1e-6;
    return (std::abs(x - point.x) < epsilon) && (std::abs(y - point.y) < epsilon);
}

PurePursuitController::PurePursuitController(
    const std::vector<Point> &path,
    double v_max,
    double d_lookahead_min,
    double d_goal_tol,
    double K_v_turn,
    double K_v_stop,
    double K_ld)
    : path_(path),
      v_max_(v_max),
      d_lookahead_min_(d_lookahead_min),
      d_goal_tol_(d_goal_tol),
      K_v_turn_(K_v_turn),
      K_v_stop_(K_v_stop),
      K_ld_(K_ld),
      v_stop_controller_(-std::abs(K_v_stop), 0.0, 0.0, 0.0) {}

std::pair<double, double> PurePursuitController::get_motion_controls(Pose current_pose, double current_linear_velocity)
{
    if (path_.empty())
        return {0.0, 0.0};

    Point goal_point_local = transform_to_local_frame(path_.back(), current_pose);
    double goal_distance = sqrt(
        pow(goal_point_local.x, 2) +
        pow(goal_point_local.y, 2));
    if (goal_distance < d_goal_tol_)
        return {0.0, 0.0};

    const double current_speed = std::abs(current_linear_velocity);
    double lookahead_distance = std::max(K_ld_ * current_speed, d_lookahead_min_);

    Point target_point_local = find_target_point(current_pose, lookahead_distance);

    double target_point_angle = atan2(target_point_local.y, target_point_local.x);
    double target_point_distance = sqrt(pow(target_point_local.x, 2) + pow(target_point_local.y, 2));
    double steering_angle;

    if (fabs(target_point_angle) < 1e-6)
        steering_angle = 0.0;
    else
    {
        double steering_radius = fabs(target_point_distance / (2 * sin(target_point_angle)));
        steering_angle = atan2(WHEEL_BASE, steering_radius);

        if (target_point_local.y < 0)
            steering_angle *= -1;

        // Define steering limit of bicycle model
        double steering_limit = atan(WHEEL_BASE / MINIMUM_TURNING_RADIUS);

        if (fabs(steering_angle) > steering_limit)
        {
            steering_angle = std::clamp(steering_angle, -steering_limit, steering_limit);
        }
            
    }

    double w;
    double v_turn = v_max_ * exp(-K_v_turn_ * fabs(steering_angle));

    if (target_point_local == goal_point_local)
    {
        w = std::min(v_turn, v_stop_controller_.get_control(target_point_distance)) / WHEEL_RADIUS;
    }
    else
    {
        w = v_turn / WHEEL_RADIUS;
    }

    return {steering_angle, w};
}

void PurePursuitController::set_path(const std::vector<Point> &path)
{
    path_ = path;
    last_closest_path_point_index_ = 0;
}

Point PurePursuitController::find_target_point(Pose current_pose, double lookahead_distance)
{
    const double epsilon = 1e-9;
    const Point current_position = current_pose.position;
    std::ostringstream debug_stream;
    debug_stream
        << "Pure pursuit target search diagnostics. "
        << "current_position=(" << current_position.x << ", " << current_position.y << "), "
        << "current_yaw=" << current_pose.orientation << ", "
        << "lookahead_distance=" << lookahead_distance;

    Point goal_point_local = transform_to_local_frame(path_.back(), current_pose);
    debug_stream
        << ", goal_point=(" << path_.back().x << ", " << path_.back().y << ")"
        << ", goal_point_local=(" << goal_point_local.x << ", " << goal_point_local.y << ")";
    if (sqrt(pow(current_position.x - path_.back().x, 2) + pow(current_position.y - path_.back().y, 2)) <=
        lookahead_distance + epsilon)
    {
        if (goal_point_local.x >= -epsilon)
            return goal_point_local;
    }

    size_t closest_path_point_index =
        find_closest_path_point_index(current_position, last_closest_path_point_index_);
    last_closest_path_point_index_ = closest_path_point_index;
    debug_stream
        << ", closest_path_point_index=" << closest_path_point_index
        << ", closest_path_point=(" << path_[closest_path_point_index].x << ", "
        << path_[closest_path_point_index].y << ")";

    const size_t first_segment_index = closest_path_point_index > 0 ? closest_path_point_index - 1 : 0;

    for (size_t i = first_segment_index; i + 1 < path_.size(); ++i)
    {
        const Point &segment_start = path_[i];
        const Point &segment_end = path_[i + 1];
        std::optional<std::vector<Point>> intersections =
            find_line_segment_circle_intersections(
                segment_start, segment_end, current_position, lookahead_distance);

        debug_stream
            << "\nsegment[" << i << "] start=(" << segment_start.x << ", " << segment_start.y
            << "), end=(" << segment_end.x << ", " << segment_end.y << ")";

        if (!intersections.has_value())
        {
            debug_stream << ", intersections=none";
            continue;
        }

        std::vector<Point> candidates = intersections.value();
        debug_stream << ", intersections=" << candidates.size();
        std::sort(
            candidates.begin(), candidates.end(),
            [&segment_start](const Point &candidate_1, const Point &candidate_2)
            {
                const double distance_1 = sqrt(
                    pow(candidate_1.x - segment_start.x, 2) +
                    pow(candidate_1.y - segment_start.y, 2));
                const double distance_2 = sqrt(
                    pow(candidate_2.x - segment_start.x, 2) +
                    pow(candidate_2.y - segment_start.y, 2));
                return distance_1 < distance_2;
            });

        for (const Point &candidate : candidates)
        {
            Point candidate_local = transform_to_local_frame(candidate, current_pose);
            debug_stream
                << "\n  candidate global=(" << candidate.x << ", " << candidate.y
                << "), local=(" << candidate_local.x << ", " << candidate_local.y
                << "), forward=" << (candidate_local.x >= -epsilon);
            if (candidate_local.x >= -epsilon)
                return candidate_local;
        }
    }

    debug_stream
        << "\nremaining_path_points:";
    for (size_t i = closest_path_point_index; i < path_.size(); ++i)
    {
        Point point_local = transform_to_local_frame(path_[i], current_pose);
        const double point_distance = hypot(
            path_[i].x - current_position.x,
            path_[i].y - current_position.y);
        debug_stream
            << "\n  path[" << i << "] global=(" << path_[i].x << ", " << path_[i].y
            << "), local=(" << point_local.x << ", " << point_local.y
            << "), distance=" << point_distance
            << ", forward=" << (point_local.x >= -epsilon);
    }
    throw std::runtime_error(debug_stream.str());
}

int PurePursuitController::find_closest_path_point_index(Point current_position, size_t start_index)
{
    double d_min = sqrt(pow(current_position.x - path_[start_index].x, 2) + pow(current_position.y - path_[start_index].y, 2));
    size_t closest_path_point_index = start_index;
    for (size_t i = start_index + 1; i < path_.size(); ++i)
    {
        double d = sqrt(pow(current_position.x - path_[i].x, 2) + pow(current_position.y - path_[i].y, 2));
        if (d < d_min)
        {
            d_min = d;
            closest_path_point_index = i;
        }
        else
            break;
    }
    return closest_path_point_index;
}

bool PurePursuitController::is_point_within_segment_bounds(Point p, Point p1, Point p2)
{
    const double epsilon = 1e-9;
    return std::min(p1.x, p2.x) - epsilon <= p.x && p.x <= std::max(p1.x, p2.x) + epsilon &&
           std::min(p1.y, p2.y) - epsilon <= p.y && p.y <= std::max(p1.y, p2.y) + epsilon;
}

std::optional<std::vector<Point>> PurePursuitController::find_line_segment_circle_intersections(Point line_point_1, Point line_point_2, Point circle_center, double circle_radius)
{
    const double epsilon = 1e-9;
    std::vector<Point> intersections;
    const double dx = line_point_2.x - line_point_1.x;
    const double dy = line_point_2.y - line_point_1.y;
    const double fx = line_point_1.x - circle_center.x;
    const double fy = line_point_1.y - circle_center.y;

    const double a = dx * dx + dy * dy;

    // A repeated path point can only intersect the circle if it lies on it.
    if (a < epsilon)
    {
        const double distance_squared = fx * fx + fy * fy;
        if (std::abs(distance_squared - circle_radius * circle_radius) < epsilon)
        {
            intersections.push_back(line_point_1);
        }
    }
    else
    {
        const double b = 2.0 * (fx * dx + fy * dy);
        const double c = fx * fx + fy * fy - circle_radius * circle_radius;
        const double discriminant = b * b - 4.0 * a * c;

        if (discriminant < -epsilon)
            return std::nullopt;

        if (std::abs(discriminant) < epsilon)
        {
            const double t = -b / (2.0 * a);
            if (t >= -epsilon && t <= 1.0 + epsilon)
            {
                intersections.emplace_back(
                    line_point_1.x + t * dx,
                    line_point_1.y + t * dy);
            }
        }
        else
        {
            const double sqrt_discriminant = sqrt(discriminant);
            const double t_1 = (-b + sqrt_discriminant) / (2.0 * a);
            const double t_2 = (-b - sqrt_discriminant) / (2.0 * a);

            if (t_1 >= -epsilon && t_1 <= 1.0 + epsilon)
            {
                intersections.emplace_back(
                    line_point_1.x + t_1 * dx,
                    line_point_1.y + t_1 * dy);
            }

            if (t_2 >= -epsilon && t_2 <= 1.0 + epsilon)
            {
                intersections.emplace_back(
                    line_point_1.x + t_2 * dx,
                    line_point_1.y + t_2 * dy);
            }
        }
    }

    if (intersections.empty())
        return std::nullopt;
    else
        return intersections;
}

Point PurePursuitController::transform_to_local_frame(Point global_point, Pose vehicle_pose)
{
    // Translate the point
    double x_t = global_point.x - vehicle_pose.position.x;
    double y_t = global_point.y - vehicle_pose.position.y;

    // Rotate the point
    double x_local = x_t * cos(vehicle_pose.orientation) + y_t * sin(vehicle_pose.orientation);
    double y_local = -x_t * sin(vehicle_pose.orientation) + y_t * cos(vehicle_pose.orientation);

    return Point(x_local, y_local);
}
