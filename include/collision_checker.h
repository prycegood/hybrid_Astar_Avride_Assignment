#pragma once

#include "state.h"
#include "occupancy_grid.h"
#include <vector>

namespace hybrid_astar {

// Robot shape parameters
struct RobotShape {
    double length;
    double width;
    double rear_axle_to_center;
    
    RobotShape() : length(4.0), width(2.0), rear_axle_to_center(1.0) {}
    RobotShape(double l, double w, double r) : length(l), width(w), rear_axle_to_center(r) {}
};

/*
 Collision checking strategy:
 1. Compute robot's four corner positions in world frame
 2. Check if any corner falls in an occupied cell
 3. Also check discretized points along the robot's perimeter
 */
class CollisionChecker {
public:
    CollisionChecker(const OccupancyGrid& grid, const RobotShape& robot, double safety_margin = 0.0);
    
    // Check if robot collides with obstacles at given state
    bool isCollision(const State& state) const;
    
    // Check if a line segment collides with obstacles
    bool isLineCollision(double x1, double y1, double x2, double y2) const;
    
    // Get robot footprint corners in world frame
    std::vector<std::pair<double, double>> getRobotFootprint(const State& state) const;
    
    // Get discretized points along robot perimeter for final collision checking
    std::vector<std::pair<double, double>> getRobotPerimeterPoints(const State& state) const;
    
private:
    const OccupancyGrid& grid_;
    RobotShape robot_;
    double safety_margin_;  // Additional clearance from obstacles (m)
    
    // Number of points to check along each edge of the robot
    int edge_discretization_;
};

}