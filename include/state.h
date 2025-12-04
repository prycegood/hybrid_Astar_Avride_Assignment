#pragma once

#include <cmath>
#include <functional>

namespace hybrid_astar {

// Represents a state in the search space: (x, y, yaw, curvature, direction)
// Discretization is done on each dimension so we can do A*
struct State {
    // Continuous state values
    double x;           // m
    double y;           // m
    double yaw;         // radians [-π, π]
    double curvature;   // rad/m (inverse turning radius)
    
    // Discretized indices for hashing
    int grid_x;
    int grid_y;
    int grid_yaw;
    int grid_curvature;
    
    // Cost values for A*
    double g;  // cost from start
    double h;  // heuristic cost to goal
    
    // Parent pointer for path reconstruction
    State* parent;
    
    // Direction (1 = forward, -1 = reverse)
    int direction;
    
    State() : x(0), y(0), yaw(0), curvature(0), 
              grid_x(0), grid_y(0), grid_yaw(0), grid_curvature(0),
              g(0), h(0), parent(nullptr), direction(1) {}
    
    State(double x_, double y_, double yaw_, double curv_, 
          double xy_resolution, double yaw_resolution, double curv_resolution);
    
    double f() const { return g + h; }
    
    // Hash function to get unique integer for the state
    size_t hash() const;
    
    // Equality operator based on discretized indices
    // Direction must be included because same (x,y,yaw,curv) in forward vs reverse are different states
    bool operator==(const State& other) const {
        return grid_x == other.grid_x && 
               grid_y == other.grid_y && 
               grid_yaw == other.grid_yaw && 
               grid_curvature == other.grid_curvature &&
               direction == other.direction;
    }
    
    // Normalize yaw to [-π, π]
    static double normalizeAngle(double angle);
};

// Hash functor for unordered_map
struct StateHash {
    size_t operator()(const State& s) const {
        return s.hash();
    }
};

// Equality functor for unordered_map
struct StateEqual {
    bool operator()(const State& a, const State& b) const {
        return a == b;
    }
};

// Configuration parameters for state discretization
struct StateConfig {
    double xy_resolution;        // m
    double yaw_resolution;       // radians
    double curvature_resolution; // rad/m
    double max_curvature;        // rad/m
    
    StateConfig() : 
        xy_resolution(0.5),
        yaw_resolution(M_PI / 36.0),
        curvature_resolution(0.1),
        max_curvature(1.0) {}
};

}