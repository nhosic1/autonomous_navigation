#include <stdexcept>
#include <optional>
#include "autonomous_navigation/common/vehicle_constants.hpp"
#include "autonomous_navigation/control/pure_pursuit.hpp"
#include <algorithm>

bool Point::operator==(const Point &point) const
{
    const double epsilon = 1e-6;
    return (std::abs(x - point.x) < epsilon) && (std::abs(y - point.y) < epsilon);
}

PurePursuitController::PurePursuitController(const std::vector<Point> &path, double v_max, double ld_min, double K_v_turn, double K_v_stop, double K_ld) : path_(path), v_max_(v_max), ld_min_(ld_min), K_v_turn_(K_v_turn), K_v_stop_(K_v_stop), K_ld_(K_ld), v_stop_controller_(K_v_stop, 0.0, 0.0, 0.0) {}

std::pair<double, double> PurePursuitController::get_motion_controls(Pose current_pose, double current_linear_velocity)
{
    if (path_.empty())
        return {0.0, 0.0};

    const double current_speed = std::abs(current_linear_velocity);
    double lookahead_distance = std::max(K_ld_ * current_speed, ld_min_);

    Point target_point = find_target_point(current_pose.position, lookahead_distance);

    if (target_point == path_.back())
    {
        double goal_distance = sqrt(pow(current_pose.position.x - path_.back().x, 2) + pow(current_pose.position.y - path_.back().y, 2));
        if (goal_distance < 0.1)
            return {0.0, 0.0};
    }

    Point target_point_local = transform_to_local_frame(target_point, current_pose);


    if (target_point_local.x < 0)
        throw std::runtime_error("Cannot compute steering angle. Target point is behind the vehicle.");

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

    if (target_point == path_.back())
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

Point PurePursuitController::find_target_point(Point current_position, double lookahead_distance)
{
    size_t closest_path_point_index = find_closest_path_point_index(current_position, last_closest_path_point_index_);
    last_closest_path_point_index_ = closest_path_point_index;

    Point closest_path_point = path_[closest_path_point_index];

    // Set index for a point outside of the lookahead distance to an invalid value
    size_t outside_path_point_index = path_.size();

    for (size_t i = closest_path_point_index; i < path_.size(); ++i)
    {
        double d = sqrt(pow(current_position.x - path_[i].x, 2) + pow(current_position.y - path_[i].y, 2));
        if (d >= lookahead_distance)
        {
            outside_path_point_index = i;
            break;
        }
    }

    // No outside point found, last path point is within the lookahead distance
    if (outside_path_point_index == path_.size())
        return path_.back();
        
    // Closest path point is outside of the lookahead radius
    if (outside_path_point_index == closest_path_point_index)
    {
        if (outside_path_point_index + 1 < path_.size())
        {
            std::optional<std::vector<Point>> intersections = find_line_segment_circle_intersections(path_[closest_path_point_index], path_[closest_path_point_index + 1], current_position, lookahead_distance);

            // Vehicle is between two path points, closest point is behind the vehicle
            if (intersections.has_value())
            {
                if (intersections.value().size() == 1)
                    return intersections.value().back();
                else
                {
                    Point intersection_1 = intersections.value()[0];
                    Point intersection_2 = intersections.value()[1];
                    double d_1 = sqrt(pow(path_[closest_path_point_index].x - intersection_1.x, 2) + pow(path_[closest_path_point_index].y - intersection_1.y, 2));
                    double d_2 = sqrt(pow(path_[closest_path_point_index].x - intersection_2.x, 2) + pow(path_[closest_path_point_index].y - intersection_2.y, 2));

                    if (d_1 >= d_2)
                        return intersection_1;
                    else
                        return intersection_2;
                }
            }
        }

        if (outside_path_point_index - 1 < path_.size())
        {
            std::optional<std::vector<Point>> intersections = find_line_segment_circle_intersections(path_[closest_path_point_index], path_[closest_path_point_index - 1], current_position, lookahead_distance);

            // Vehicle is between two path points, closest point is in front of the vehicle
            if (intersections.has_value())
            {
                if (intersections.value().size() == 1)
                    return intersections.value().back();
                else
                {
                    Point intersection_1 = intersections.value()[0];
                    Point intersection_2 = intersections.value()[1];
                    double d_1 = sqrt(pow(path_[closest_path_point_index].x - intersection_1.x, 2) + pow(path_[closest_path_point_index].y - intersection_1.y, 2));
                    double d_2 = sqrt(pow(path_[closest_path_point_index].x - intersection_2.x, 2) + pow(path_[closest_path_point_index].y - intersection_2.y, 2));

                    if (d_1 < d_2)
                        return intersection_1;
                    else
                        return intersection_2;
                }
            }
        }
        return closest_path_point;
    }

    // Closest path point is inside the lookahead radius
    // Look for the target point between two path points closest to the lookahead radius
    std::optional<std::vector<Point>> intersections = find_line_segment_circle_intersections(path_[outside_path_point_index - 1], path_[outside_path_point_index], current_position, lookahead_distance);
    if (intersections.has_value())
    {
        if (intersections.value().size() == 1)
            return intersections.value().back();
        
        throw std::runtime_error("Found two target point candidates, but only one is expected.");    
    }

    throw std::runtime_error("No target point found within the lookahead distance");
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

bool PurePursuitController::is_point_on_line_segment(Point p, Point p1, Point p2)
{
    return std::min(p1.x, p2.x) <= p.x && p.x <= std::max(p1.x, p2.x) && std::min(p1.y, p2.y) <= p.y && p.y <= std::max(p1.y, p2.y);
}

std::optional<std::vector<Point>> PurePursuitController::find_line_segment_circle_intersections(Point line_point_1, Point line_point_2, Point circle_center, double circle_radius)
{
    double m = (line_point_2.y - line_point_1.y) / (line_point_2.x - line_point_1.x);
    double c = line_point_1.y - line_point_1.x * m;

    double A = 1 + m * m;
    double B = 2 * m * (c - circle_center.y) - 2 * circle_center.x;
    double C = pow(circle_center.x, 2) + pow(c - circle_center.y, 2) - pow(circle_radius, 2);

    double discriminant = B * B - 4 * A * C;

    if (discriminant < 0)
        return std::nullopt;

    std::vector<Point> intersections;

    if (discriminant == 0)
    {
        double x = -B / (2 * A);
        double y = m * x + c;
        Point intersection(x, y);

        if (is_point_on_line_segment(intersection, line_point_1, line_point_2))
            intersections.push_back(intersection);
    }
    else
    {
        double x_1 = (-B + sqrt(discriminant)) / (2 * A);
        double y_1 = m * x_1 + c;
        Point intersection_1(x_1, y_1);

        if (is_point_on_line_segment(intersection_1, line_point_1, line_point_2))
        {
            intersections.push_back(intersection_1);
        }

        double x_2 = (-B - sqrt(discriminant)) / (2 * A);
        double y_2 = m * x_2 + c;
        Point intersection_2(x_2, y_2);

        if (is_point_on_line_segment(intersection_2, line_point_1, line_point_2))
        {
            intersections.push_back(intersection_2);
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
