#include "collision_checker.h"
#include <cmath>
#include <iostream>

namespace hybrid_astar {

CollisionChecker::CollisionChecker(const OccupancyGrid& grid, const RobotShape& robot, double safety_margin)
    : grid_(grid), robot_(robot), safety_margin_(safety_margin), edge_discretization_(5) {
}

bool CollisionChecker::isCollision(const State& state) const {
    // Get all perimeter points and check each one
    auto points = getRobotPerimeterPoints(state);
    
    for (const auto& point : points) {
        if (grid_.isOccupied(point.first, point.second)) {
            return true;
        }
    }
    
    return false;
}

bool CollisionChecker::isLineCollision(double x1, double y1, double x2, double y2) const {
    // Walking along the line segment and checking if any points are occupied
    double dx = x2 - x1;
    double dy = y2 - y1;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    if (distance < 0.01) return false;
    
    int num_checks = static_cast<int>(distance / grid_.getResolution()) + 1;
    
    for (int i = 0; i <= num_checks; ++i) {
        double t = static_cast<double>(i) / num_checks;
        double x = x1 + t * dx;
        double y = y1 + t * dy;
        
        if (grid_.isOccupied(x, y)) {
            return true;
        }
    }
    
    return false;
}

std::vector<std::pair<double, double>> 
CollisionChecker::getRobotFootprint(const State& state) const {
    std::vector<std::pair<double, double>> corners;
    
    double cos_yaw = std::cos(state.yaw);
    double sin_yaw = std::sin(state.yaw);
    
    // Robot frame: x is forward and y is left
    // Corners relative to rear axle
    // I add a safety margin to all dimensions for increased clearance
    double front = robot_.length - robot_.rear_axle_to_center + safety_margin_;
    double rear = -robot_.rear_axle_to_center - safety_margin_;
    double left = robot_.width / 2.0 + safety_margin_;
    double right = -robot_.width / 2.0 - safety_margin_;
    
    // Transform corners to world frame
    auto transformPoint = [&](double x_robot, double y_robot) {
        double x_world = state.x + x_robot * cos_yaw - y_robot * sin_yaw;
        double y_world = state.y + x_robot * sin_yaw + y_robot * cos_yaw;
        return std::make_pair(x_world, y_world);
    };
    
    corners.push_back(transformPoint(front, left));   // front-left
    corners.push_back(transformPoint(front, right));  // front-right
    corners.push_back(transformPoint(rear, right));   // rear-right
    corners.push_back(transformPoint(rear, left));    // rear-left
    
    return corners;
}

std::vector<std::pair<double, double>> 
CollisionChecker::getRobotPerimeterPoints(const State& state) const {
    std::vector<std::pair<double, double>> points;
    
    auto corners = getRobotFootprint(state);
    
    // Add corners
    for (const auto& corner : corners) {
        points.push_back(corner);
    }
    
    // Add discretized points along edges
    for (size_t i = 0; i < corners.size(); ++i) {
        size_t next_i = (i + 1) % corners.size();
        
        double x1 = corners[i].first;
        double y1 = corners[i].second;
        double x2 = corners[next_i].first;
        double y2 = corners[next_i].second;
        
        for (int j = 1; j < edge_discretization_; ++j) {
            double t = static_cast<double>(j) / edge_discretization_;
            double x = x1 + t * (x2 - x1);
            double y = y1 + t * (y2 - y1);
            points.push_back(std::make_pair(x, y));
        }
    }
    
    // Also check center points to be safe
    double cos_yaw = std::cos(state.yaw);
    double sin_yaw = std::sin(state.yaw);
    
    int num_interior = 5;
    for (int i = 0; i <= num_interior; ++i) {
        double x_robot = -robot_.rear_axle_to_center + 
                        i * robot_.length / num_interior;
        for (int j = 0; j <= num_interior; ++j) {
            double y_robot = -robot_.width / 2.0 + j * robot_.width / num_interior;
            
            double x_world = state.x + x_robot * cos_yaw - y_robot * sin_yaw;
            double y_world = state.y + x_robot * sin_yaw + y_robot * cos_yaw;
            points.push_back(std::make_pair(x_world, y_world));
        }
    }
    
    return points;
}

}


