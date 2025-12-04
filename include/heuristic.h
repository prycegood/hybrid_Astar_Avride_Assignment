#pragma once

#include "state.h"
#include "occupancy_grid.h"
#include <vector>
#include <limits>

namespace hybrid_astar {

// Heuristic calculator for Hybrid A*
// I use a Voronoi-like field to combine goal distance and obstacle distance as my final heuristic
class Heuristic {
public:
    Heuristic(const OccupancyGrid& grid, const State& goal);
    
    // Compute heuristic cost from current state to goal
    double compute(const State& current) const;
    
    // Precompute Voronoi-like field: goal distance + obstacle distance transform
    void precomputeHolonomicHeuristic();
    
    // Get value from final Voronoi-like field
    double getHolonomicDistance(double x, double y) const;
    
    // Simple Euclidean distance
    static double euclideanDistance(const State& a, const State& b);
    
    // Simplified Reeds-Shepp distance approximation
    static double nonholonomicDistance(const State& a, const State& b, double max_curvature);
    
private:
    const OccupancyGrid& grid_;
    State goal_;
    
    // Voronoi field components
    std::vector<std::vector<double>> goal_distance_field_;     // Distance to goal
    std::vector<std::vector<double>> obstacle_distance_field_; // Distance to nearest obstacle
    std::vector<std::vector<double>> voronoi_field_;          // Combined field
    bool heuristic_computed_;
    
    // Voronoi field parameters
    double obstacle_weight_;      // Weight for obstacle avoidance
    double robot_clearance_;      // Minimum desired clearance from obstacles
    
    void computeObstacleDistanceTransform();
    
    void computeGoalDistanceField();
    
    //Combine goal and obstacle fields into final Voronoi-like field
    void computeVoronoiField();
};

}