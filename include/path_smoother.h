#pragma once

#include "state.h"
#include "occupancy_grid.h"
#include "collision_checker.h"
#include <vector>

namespace hybrid_astar {

// Path smoother uses gradient descent to smooth the path and interpolate it to create uniform waypoint spacing
class PathSmoother {
public:
    PathSmoother(const OccupancyGrid& grid, const RobotShape& robot);

    // Smooth the path using gradient descent
    std::vector<State> smoothPath(const std::vector<State>& path, int iterations = 50);
    
    // Interpolate path to create uniform waypoint spacing
    std::vector<State> interpolatePath(const std::vector<State>& path, double spacing = 0.5);
    
    // Smooth and interpolate the path
    std::vector<State> smoothAndInterpolate(const std::vector<State>& path, 
                                            int cg_iterations = 50,
                                            double interpolation_spacing = 0.5);
    
private:
    std::unique_ptr<CollisionChecker> collision_checker_;
    
    // Recompute yaw values to match path direction
    void recomputeYawValues(std::vector<State>& path) const;
    
    // Compute curvature at waypoint i
    double computeCurvature(const std::vector<State>& path, size_t i) const;
    
    // Compute gradient of smoothness cost at waypoint i
    void computeGradient(const std::vector<State>& path, size_t i, 
                        double& grad_x, double& grad_y) const;
    
    // Simple gradient descent step
    void gradientDescentStep(std::vector<State>& path, double step_size) const;
    
    // Interpolate between two states
    State interpolateState(const State& from, const State& to, double t) const;
};

}