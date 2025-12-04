#include "hybrid_astar.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <algorithm>

namespace hybrid_astar {

HybridAStar::HybridAStar(const OccupancyGrid& grid, 
                         const RobotShape& robot,
                         const StateConfig& config,
                         double safety_margin)
    : grid_(grid), config_(config), step_size_(1.0) {
    
    collision_checker_ = std::make_unique<CollisionChecker>(grid, robot, safety_margin);
    
    // Will initialize heuristic when we know the goal
    heuristic_ = nullptr;
    
    // Define curvature changes for motion primitives
    // These represent the steering rate limit
    double max_curvature_change = 0.15; // After testing, I found that .15 is good for this project
    curvature_changes_ = {
        -max_curvature_change, 
        -max_curvature_change / 2.0, 
        0.0, 
        max_curvature_change / 2.0, 
        max_curvature_change
    };
}

PlanResult HybridAStar::plan(const State& start, const State& goal, double max_planning_time_ms) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    PlanResult result;
    result.success = false;
    result.nodes_expanded = 0;
    
    // Initialize heuristic with goal
    heuristic_ = std::make_unique<Heuristic>(grid_, goal);
    std::cout << "Precomputing holonomic heuristic..." << std::endl;
    heuristic_->precomputeHolonomicHeuristic();
    std::cout << "Heuristic precomputation complete." << std::endl;
    
    // Initialize start state
    State start_state(start.x, start.y, start.yaw, start.curvature,
                     config_.xy_resolution, config_.yaw_resolution, config_.curvature_resolution);
    start_state.g = 0.0;
    start_state.h = heuristic_->compute(start_state);
    start_state.parent = nullptr;
    start_state.direction = 1;
    
    // Priority queue (open set)
    std::priority_queue<State*, std::vector<State*>, StateComparator> open_set;
    
    // Hash map for closed set and finding states
    std::unordered_map<size_t, State*> state_map;
    
    // Allocate start state on heap
    State* start_ptr = new State(start_state);
    open_set.push(start_ptr);
    state_map[start_ptr->hash()] = start_ptr;
    
    State* goal_state_ptr = nullptr;
    
    // Goal tolerance
    // Tight tolerances for realistic parking
    double goal_xy_tolerance = 0.5;  // m (within 0.5m of goal position)
    double goal_yaw_tolerance = 0.175;  // radians (about 10 degrees)
    
    std::cout << "Starting search..." << std::endl;
    std::cout << "Start: (" << start.x << ", " << start.y << ", " << start.yaw << ")" << std::endl;
    std::cout << "Goal: (" << goal.x << ", " << goal.y << ", " << goal.yaw << ")" << std::endl;
    
    while (!open_set.empty()) {
        // Check time limit
        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(current_time - start_time).count();
        if (elapsed_ms > max_planning_time_ms) {
            std::cout << "Planning timeout after " << elapsed_ms << " ms" << std::endl;
            break;
        }
        
        // Get state with lowest f-score
        State* current = open_set.top();
        open_set.pop();
        
        result.nodes_expanded++;
        
        // Store explored state for visualization
        if (result.nodes_expanded % 10 == 0) {  // Sample every 10th state to reduce memory
            result.explored_states.push_back(*current);
        }
        
        // Progress output
        if (result.nodes_expanded % 1000 == 0) {
            std::cout << "Looked at " << result.nodes_expanded << " nodes, "
                     << "current f-score: " << current->f() 
                     << ", open set size: " << open_set.size() << std::endl;
        }
        
        // Check if goal reached
        double dx = current->x - goal.x;
        double dy = current->y - goal.y;
        double dist_to_goal = std::sqrt(dx * dx + dy * dy);
        double yaw_diff = std::abs(State::normalizeAngle(current->yaw - goal.yaw));
        
        if (dist_to_goal < goal_xy_tolerance && yaw_diff < goal_yaw_tolerance) {
            std::cout << "Goal reached. Distance: " << dist_to_goal << ", yaw diff: " << yaw_diff << std::endl;
            goal_state_ptr = current;
            break;
        }
        
        // Generate successors
        auto motion_primitives = getMotionPrimitives(*current, goal);
        
        static int collision_sample_counter = 0;
        for (const auto& motion : motion_primitives) {
            State successor = applyMotion(*current, motion);
            
            // Check validity of the final state
            if (!isValid(successor)) {
                // Track collision state (sample every 50th to avoid clutter)
                collision_sample_counter++;
                if (collision_sample_counter % 50 == 0) {
                    result.collision_states.push_back(successor);
                }
                continue;
            }
            
            // Check if the trajectory from current to successor is collision-free
            if (!isTrajectoryValid(*current, successor)) {
                // Track collision state (sample every 50th)
                collision_sample_counter++;
                if (collision_sample_counter % 50 == 0) {
                    result.collision_states.push_back(successor);
                }
                continue;
            }
            
            // Compute cost
            double transition_cost = transitionCost(*current, successor);
            double tentative_g = current->g + transition_cost;
            
            // Check if this state was already visited
            size_t succ_hash = successor.hash();
            auto it = state_map.find(succ_hash);
            
            if (it != state_map.end()) {
                // State already exists so this means we found a better path
                State* existing = it->second;
                if (tentative_g < existing->g) {
                    // Check if new trajectory reaches cell at similar continuous position
                    // Updating coordinates can break path reconstruction if positions differ by a lot due to hashing collisions
                    double dx = successor.x - existing->x;
                    double dy = successor.y - existing->y;
                    double position_diff = std::sqrt(dx*dx + dy*dy);
                    
                    // Only update if positions are very close (within discretization tolerance)
                    // Otherwise we risk creating large jumps in the reconstructed path
                    if (position_diff < config_.xy_resolution * 0.6) {  // After testing, I found that .6 was giving good results
                        existing->x = successor.x;
                        existing->y = successor.y;
                        existing->yaw = successor.yaw;
                        existing->curvature = successor.curvature;
                        existing->g = tentative_g;
                        existing->parent = current;
                        open_set.push(existing);
                    }
                }
            } else {
                // New state
                successor.g = tentative_g;
                successor.h = heuristic_->compute(successor);
                successor.parent = current;
                
                State* succ_ptr = new State(successor);
                state_map[succ_hash] = succ_ptr;
                open_set.push(succ_ptr);
            }
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    result.planning_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    if (goal_state_ptr != nullptr) {
        result.success = true;
        result.path = extractPath(goal_state_ptr);
        std::cout << "Path found with " << result.path.size() << " waypoints" << std::endl;
    } else {
        std::cout << "No path found" << std::endl;
    }
    
    std::cout << "Planning complete: " << result.nodes_expanded << " nodes expanded in " 
              << result.planning_time_ms << " ms" << std::endl;
    
    // Clean up allocated states
    for (auto& pair : state_map) {
        delete pair.second;
    }
    
    return result;
}

std::vector<MotionPrimitive> HybridAStar::getMotionPrimitives(const State& current, const State& goal) const {
    std::vector<MotionPrimitive> primitives;
    
    // Always offer motions in the current direction
    int current_direction = (current.direction == 0) ? 1 : current.direction;  // Start defaults to forward
    
    for (double delta_curv : curvature_changes_) {
        primitives.push_back({step_size_, delta_curv, current_direction});
    }
    
    // Calculate distance to goal
    double dx = current.x - goal.x;
    double dy = current.y - goal.y;
    double dist_to_goal = std::sqrt(dx * dx + dy * dy);
    
    // It was taking forever to run, so I only allow the robot to reverse at the start and near the goal
    if (current.g < 0.1 || current_direction < 0 || dist_to_goal < 10.0) {
        // Standard reverse motions (move while changing curvature gradually)
        for (double delta_curv : {curvature_changes_[0], curvature_changes_[2], curvature_changes_[4]}) {
            primitives.push_back({step_size_, delta_curv, -current_direction});
        }
        
        // This is for gear changing when stationary
        for (double target_curv : {-config_.max_curvature, -config_.max_curvature/2, 0.0, 
                                     config_.max_curvature/2, config_.max_curvature}) {
            double delta = target_curv - current.curvature;
            // Only add if this is a significant steering change
            if (std::abs(delta) > 0.1) {
                primitives.push_back({0.0, delta, -current_direction});
            }
        }
    }
    
    return primitives;
}

State HybridAStar::applyMotion(const State& current, const MotionPrimitive& motion) const {
    // Bicycle model kinematics
    State next = current;
    next.direction = motion.direction;
    
    // Update curvature
    next.curvature = current.curvature + motion.delta_curvature;
    next.curvature = std::max(-config_.max_curvature, 
                             std::min(config_.max_curvature, next.curvature));
    
    // This is for gear changing when stationary
    if (motion.distance < 0.01) {
        next.x = current.x;
        next.y = current.y;
        next.yaw = current.yaw;
        // Curvature and direction are already updated above
    } else {
        // This is for moving
        double avg_curvature = (current.curvature + next.curvature) / 2.0;
        
        // This is for straight line motion
        if (std::abs(avg_curvature) < 1e-4) {
            next.x = current.x + motion.direction * motion.distance * std::cos(current.yaw);
            next.y = current.y + motion.direction * motion.distance * std::sin(current.yaw);
            next.yaw = current.yaw;
        } else {
            // Arc motion
            double delta_yaw = motion.direction * motion.distance * avg_curvature;
            next.yaw = State::normalizeAngle(current.yaw + delta_yaw);
            double radius = 1.0 / avg_curvature;
            next.x = current.x + motion.direction * radius * (std::sin(next.yaw) - std::sin(current.yaw));
            next.y = current.y - motion.direction * radius * (std::cos(next.yaw) - std::cos(current.yaw));
        }
    }
    
    // Update discretized indices
    next.grid_x = static_cast<int>(std::round(next.x / config_.xy_resolution));
    next.grid_y = static_cast<int>(std::round(next.y / config_.xy_resolution));
    next.grid_yaw = static_cast<int>(std::round(next.yaw / config_.yaw_resolution));
    next.grid_curvature = static_cast<int>(std::round(next.curvature / config_.curvature_resolution));
    
    return next;
}

bool HybridAStar::isValid(const State& state) const {
    // Check bounds
    if (state.x < 0 || state.x >= grid_.getWidth() ||
        state.y < 0 || state.y >= grid_.getHeight()) {
        return false;
    }
    
    // Check collision
    return !collision_checker_->isCollision(state);
}

bool HybridAStar::isTrajectoryValid(const State& from, const State& to) const {
    // Sample along the trajectory from 'from' to 'to' states and check collisions
    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // Stationary check
    if (distance < 0.01) {
        return true;
    }
    
    int num_samples = static_cast<int>(std::ceil(distance / 0.15));
    
    // Check samples along trajectory
    for (int i = 1; i <= num_samples; ++i) {
        double t = static_cast<double>(i) / num_samples;
        
        // Interpolate position and yaw
        State intermediate;
        intermediate.x = from.x + t * dx;
        intermediate.y = from.y + t * dy;
        
        // Interpolate yaw (handle angle wraparound)
        double yaw_diff = State::normalizeAngle(to.yaw - from.yaw);
        intermediate.yaw = State::normalizeAngle(from.yaw + t * yaw_diff);
        
        // Interpolate curvature
        intermediate.curvature = from.curvature + t * (to.curvature - from.curvature);
        intermediate.direction = to.direction;  // Use destination direction for footprint checking
        
        // Check if this intermediate state collides
        if (!isValid(intermediate)) {
            return false;
        }
    }
    
    return true;
}

double HybridAStar::distanceToNearestObstacle(double x, double y) const {
    // Sample in a radius around the position to find nearest obstacle
    double search_radius = 3.0;  // m - how far to look for obstacles
    double sample_resolution = 0.3;  // m - how finely to sample
    
    double min_distance = search_radius;
    
    // Sample in a grid pattern around the position
    int num_samples = static_cast<int>(search_radius / sample_resolution);
    
    for (int dx = -num_samples; dx <= num_samples; ++dx) {
        for (int dy = -num_samples; dy <= num_samples; ++dy) {
            double sample_x = x + dx * sample_resolution;
            double sample_y = y + dy * sample_resolution;
            
            // Check if this point is occupied
            if (grid_.isOccupied(sample_x, sample_y)) {
                double dist = std::sqrt(dx * dx + dy * dy) * sample_resolution;
                min_distance = std::min(min_distance, dist);
            }
        }
    }
    
    return min_distance;
}

// All weights in this method were tuned by testing the code and they seem to work well
double HybridAStar::transitionCost(const State& from, const State& to) const {
    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // Gear change check
    if (distance < 0.01) {
        double cost = 2.0;
        
        // Direction change penalty (gear shift)
        if (from.direction != to.direction) {
            cost += cost_weights.direction_change_penalty * 0.5;
        }
        
        // Small penalty for large steering adjustments (takes time)
        double curvature_change = std::abs(to.curvature - from.curvature);
        cost += curvature_change * 2.0;
        
        return cost;
    }
    
    // Car is moving
    double cost = distance;
    
    // Reverse penalty
    if (to.direction < 0) {
        cost *= cost_weights.reverse_penalty;
    }
    
    // Direction change penalty just to be safe even though it shouldn't happen while moving
    if (from.direction != to.direction) {
        cost += cost_weights.direction_change_penalty;
    }
    
    // Curvature penalty (prefer straighter paths)
    double avg_curvature = (std::abs(from.curvature) + std::abs(to.curvature)) / 2.0;
    cost += cost_weights.curvature_penalty * avg_curvature * distance;
    
    // Curvature change penalty (prefer smooth paths)
    double curvature_change = std::abs(to.curvature - from.curvature);
    cost += cost_weights.curvature_change_penalty * curvature_change;
    
    // Obstacle proximity penalty (reward keeping distance from walls)
    double obstacle_dist = distanceToNearestObstacle(to.x, to.y);
    if (obstacle_dist < cost_weights.safe_distance) {
        // Linear penalty as we get closer to obstacles
        double clearance_ratio = obstacle_dist / cost_weights.safe_distance;
        double proximity_cost = cost_weights.obstacle_proximity_penalty * 
                               (1.0 - clearance_ratio) * distance;
        cost += proximity_cost;
    }
    
    return cost;
}

std::vector<State> HybridAStar::extractPath(State* goal_state) const {
    std::vector<State> path;
    
    State* current = goal_state;
    while (current != nullptr) {
        path.push_back(*current);
        current = current->parent;
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

}


