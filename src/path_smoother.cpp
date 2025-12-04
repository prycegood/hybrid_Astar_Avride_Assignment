#include "path_smoother.h"
#include <cmath>
#include <iostream>

namespace hybrid_astar {

PathSmoother::PathSmoother(const OccupancyGrid& grid, const RobotShape& robot) {
    collision_checker_ = std::make_unique<CollisionChecker>(grid, robot);
}

std::vector<State> PathSmoother::smoothPath(const std::vector<State>& path, int iterations) {
    if (path.size() < 3) {
        return path;  // Nothing to smooth
    }
    
    std::vector<State> smoothed = path;
    
    // Gradient descent to minimize curvature
    double step_size = 0.1;  // meters per iteration
    
    for (int iter = 0; iter < iterations; ++iter) {
        // Adaptive step size
        if (iter > iterations / 2) {
            step_size = 0.05;  // Smaller steps for fine-tuning
        }
        
        gradientDescentStep(smoothed, step_size);
    }
    
    return smoothed;
}

std::vector<State> PathSmoother::interpolatePath(const std::vector<State>& path, double spacing) {
    if (path.size() < 2) {
        return path;
    }
    
    std::vector<State> interpolated;
    interpolated.push_back(path[0]);  // Always include start
    
    for (size_t i = 0; i < path.size() - 1; ++i) {
        const State& from = path[i];
        const State& to = path[i + 1];
        
        // Should not interpolate across direction changes
        bool crosses_direction_change = (from.direction != to.direction);
        
        if (crosses_direction_change) {
            if (i < path.size() - 2) {
                interpolated.push_back(to);
            }
            continue;
        }
        
        // Compute distance between waypoints
        double dx = to.x - from.x;
        double dy = to.y - from.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        // Number of intermediate points needed
        int num_points = static_cast<int>(std::ceil(distance / spacing));
        
        // Check if the entire segment is collision-free
        bool segment_clear = true;
        for (int j = 1; j < num_points; ++j) {
            double t = static_cast<double>(j) / num_points;
            State test_state = interpolateState(from, to, t);
            if (collision_checker_->isCollision(test_state)) {
                segment_clear = false;
                break;
            }
        }
        
        // Only interpolate if the entire segment is collision-free
        if (segment_clear) {
            for (int j = 1; j < num_points; ++j) {
                double t = static_cast<double>(j) / num_points;
                State interp = interpolateState(from, to, t);
                interpolated.push_back(interp);
            }
        }

        // If segment has collisions, skip interpolation
        if (i < path.size() - 2) {
            interpolated.push_back(to);
        }
    }
    
    interpolated.push_back(path.back());
    
    return interpolated;
}

std::vector<State> PathSmoother::smoothAndInterpolate(const std::vector<State>& path, 
                                                       int cg_iterations,
                                                       double interpolation_spacing) {
    std::cout << "  Phase 2 smoothing: " << path.size() << " waypoints" << std::endl;
    
    // Save the exact start and goal states to restore later
    State exact_start = path.front();
    State exact_goal = path.back();
    
    // Smooth using gradient descent
    std::vector<State> smoothed = smoothPath(path, cg_iterations);
    std::cout << "  After smoothing: " << smoothed.size() << " waypoints" << std::endl;
    
    // Recompute yaw values to match smoothed path direction
    recomputeYawValues(smoothed);
    
    // Interpolate for uniform spacing
    std::vector<State> interpolated = interpolatePath(smoothed, interpolation_spacing);
    std::cout << "  After interpolation: " << interpolated.size() << " waypoints" << std::endl;
    
    // Recompute yaw values again after interpolation
    recomputeYawValues(interpolated);
    
    // Restore exact start and goal states
    if (!interpolated.empty()) {
        interpolated.front() = exact_start;
        interpolated.back() = exact_goal;
        std::cout << "  Start/Goal states restored for exact match" << std::endl;
    }
    
    return interpolated;
}

double PathSmoother::computeCurvature(const std::vector<State>& path, size_t i) const {
    if (i == 0 || i >= path.size() - 1) {
        return 0.0;  // No curvature at endpoints
    }
    
    // Use three points to estimate curvature
    const State& prev = path[i - 1];
    const State& curr = path[i];
    const State& next = path[i + 1];
    
    // Vectors
    double dx1 = curr.x - prev.x;
    double dy1 = curr.y - prev.y;
    double dx2 = next.x - curr.x;
    double dy2 = next.y - curr.y;
    
    // Normalize
    double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
    
    if (len1 < 1e-6 || len2 < 1e-6) {
        return 0.0;
    }
    
    dx1 /= len1;
    dy1 /= len1;
    dx2 /= len2;
    dy2 /= len2;
    
    // Cross product for signed curvature
    double cross = dx1 * dy2 - dy1 * dx2;
    double curvature = 2.0 * cross / (len1 + len2);
    
    return curvature;
}

void PathSmoother::computeGradient(const std::vector<State>& path, size_t i,
                                   double& grad_x, double& grad_y) const {
    grad_x = 0.0;
    grad_y = 0.0;
    
    // Don't move endpoints
    if (i == 0 || i >= path.size() - 1) {
        return;
    }
    
    const State& prev = path[i - 1];
    const State& curr = path[i];
    const State& next = path[i + 1];
    
    // Smoothness term (pull toward average of neighbors)
    double avg_x = (prev.x + next.x) / 2.0;
    double avg_y = (prev.y + next.y) / 2.0;
    
    grad_x = avg_x - curr.x;
    grad_y = avg_y - curr.y;
    
    // Weight by curvature (higher curvature = more smoothing needed)
    double curvature = std::abs(computeCurvature(path, i));
    double weight = 1.0 + curvature * 5.0;
    
    grad_x *= weight;
    grad_y *= weight;
}

void PathSmoother::gradientDescentStep(std::vector<State>& path, double step_size) const {
    // Only move interior points
    for (size_t i = 1; i < path.size() - 1; ++i) {
        // Do not smooth waypoints where direction changes occur
        bool is_direction_change = (path[i].direction != path[i-1].direction) || 
                                    (path[i].direction != path[i+1].direction);
        
        if (is_direction_change) {
            continue;
        }
        
        double grad_x, grad_y;
        computeGradient(path, i, grad_x, grad_y);
        
        // Proposed new position
        State new_state = path[i];
        new_state.x += step_size * grad_x;
        new_state.y += step_size * grad_y;
        
        // Check if new position is collision-free
        if (!collision_checker_->isCollision(new_state)) {
            // Also check that segments to/from new position are collision-free
            bool prev_segment_clear = !collision_checker_->isLineCollision(
                path[i-1].x, path[i-1].y, new_state.x, new_state.y);
            bool next_segment_clear = !collision_checker_->isLineCollision(
                new_state.x, new_state.y, path[i+1].x, path[i+1].y);
            
            if (prev_segment_clear && next_segment_clear) {
                path[i] = new_state;
            }
        }
    }
}

State PathSmoother::interpolateState(const State& from, const State& to, double t) const {
    State result;
    
    // Linear interpolation of position
    result.x = from.x + t * (to.x - from.x);
    result.y = from.y + t * (to.y - from.y);
    
    // Interpolate yaw (handle wraparound)
    double yaw_diff = State::normalizeAngle(to.yaw - from.yaw);
    result.yaw = State::normalizeAngle(from.yaw + t * yaw_diff);
    
    // Interpolate curvature
    result.curvature = from.curvature + t * (to.curvature - from.curvature);
    
    // Keep direction consistent
    result.direction = from.direction;
    
    // Update grid indices (not critical for interpolated points)
    result.grid_x = static_cast<int>(std::round(result.x / 0.5));
    result.grid_y = static_cast<int>(std::round(result.y / 0.5));
    result.grid_yaw = static_cast<int>(std::round(result.yaw / (M_PI / 36)));
    result.grid_curvature = static_cast<int>(std::round(result.curvature / 0.1));
    
    // Copy cost values
    result.g = from.g + t * (to.g - from.g);
    result.h = from.h + t * (to.h - from.h);
    result.parent = nullptr;
    
    return result;
}

void PathSmoother::recomputeYawValues(std::vector<State>& path) const {
    if (path.size() < 2) return;
    
    // Find yaw for all waypoints except the last
    for (size_t i = 0; i < path.size() - 1; ++i) {
        double dx = path[i+1].x - path[i].x;
        double dy = path[i+1].y - path[i].y;
        double dist = std::sqrt(dx*dx + dy*dy);
        
        // Check if waypoints are not too close
        if (dist > 0.01) {
            double movement_angle = std::atan2(dy, dx);
            
            // When reversing, car front points opposite to movement direction
            if (path[i].direction < 0) {
                path[i].yaw = State::normalizeAngle(movement_angle + M_PI);
            } else {
                path[i].yaw = movement_angle;
            }
        }
    }
    
    // For the last waypoint, use the same logic
    if (path.size() >= 2) {
        double dx = path[path.size()-1].x - path[path.size()-2].x;
        double dy = path[path.size()-1].y - path[path.size()-2].y;
        double dist = std::sqrt(dx*dx + dy*dy);
        
        if (dist > 0.01) {
            double movement_angle = std::atan2(dy, dx);
            
            // Apply same direction logic for last waypoint
            if (path.back().direction < 0) {
                path.back().yaw = State::normalizeAngle(movement_angle + M_PI);
            } else {
                path.back().yaw = movement_angle;
            }
        }
    }
}

}