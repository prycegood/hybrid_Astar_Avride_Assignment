#pragma once

#include "state.h"
#include "occupancy_grid.h"
#include "collision_checker.h"
#include "heuristic.h"
#include <vector>
#include <unordered_map>
#include <queue>
#include <memory>

namespace hybrid_astar {

// Motion primitive for state expansion
struct MotionPrimitive {
    double distance;  // arc length
    double delta_curvature;  // change in curvature
    int direction;  // 1 for forward, -1 for reverse
};
// Planning result
struct PlanResult {
    bool success;
    std::vector<State> path;
    int nodes_expanded;
    double planning_time_ms;
    std::vector<State> explored_states;  // For visualization
    std::vector<State> collision_states;  // States rejected due to collision
};

// Hybrid A* path planner
class HybridAStar {
public:
    HybridAStar(const OccupancyGrid& grid, 
                const RobotShape& robot,
                const StateConfig& config,
                double safety_margin = 0.5);  // Safety margin from obstacles (m)
    
    // Plan path from start to goal
    PlanResult plan(const State& start, const State& goal, double max_planning_time_ms = 5000.0);
    
    // Generate motion primitives for state expansion
    std::vector<MotionPrimitive> getMotionPrimitives(const State& current, const State& goal) const;
    
    // Apply motion primitive to get next state. Uses bicycle model kinematics
    State applyMotion(const State& current, const MotionPrimitive& motion) const;
    
    // Check if state is valid (collision-free and within bounds)
    bool isValid(const State& state) const;
    
    // Check if trajectory from state A to state B is collision-free
    bool isTrajectoryValid(const State& from, const State& to) const;
    
    // Compute distance to nearest obstacle from a given position
    double distanceToNearestObstacle(double x, double y) const;
    
    // Compute cost of transitioning from one state to another
    double transitionCost(const State& from, const State& to) const;
    
    // Extract path from goal state by following parent pointers
    std::vector<State> extractPath(State* goal_state) const;
    
    // Cost weights (I tuned them by testing the code and they seem to work well)
    struct CostWeights {
        double reverse_penalty = 2.5;
        double direction_change_penalty = 30.0;
        double curvature_penalty = 3.0;
        double curvature_change_penalty = 5.0;
        double obstacle_proximity_penalty = 30.0;
        double safe_distance = 2.0;
    } cost_weights;
    
private:
    const OccupancyGrid& grid_;
    StateConfig config_;
    std::unique_ptr<CollisionChecker> collision_checker_;
    std::unique_ptr<Heuristic> heuristic_;
    
    // Motion primitive generation parameters
    double step_size_;  // arc length per primitive (m)
    std::vector<double> curvature_changes_;  // discretized curvature changes
    
    // Comparator for priority queue (min-heap based on f-score)
    struct StateComparator {
        bool operator()(const State* a, const State* b) const {
            return a->f() > b->f();
        }
    };
};

}