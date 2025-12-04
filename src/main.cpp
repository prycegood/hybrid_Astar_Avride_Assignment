#include "state.h"
#include "occupancy_grid.h"
#include "collision_checker.h"
#include "heuristic.h"
#include "hybrid_astar.h"
#include "path_smoother.h"
#include "mcap_writer.h"
#include <iostream>
#include <memory>

using namespace hybrid_astar;

int main(int argc, char** argv) {
    // Configuration
    double grid_width = 80.0;   // meters
    double grid_height = 80.0;  // meters
    double grid_resolution = 0.15;  // meters
    
    std::cout << "Creating occupancy grid (" << grid_width << "m x " << grid_height << "m, " 
              << grid_resolution << "m resolution)..." << std::endl;
    
    // Create occupancy grid
    OccupancyGrid grid(grid_width, grid_height, grid_resolution);
    
    // Choose scenario
    std::string scenario = "complex";  // Options are "simple" or "complex"
    
    if (argc > 1) {
        scenario = argv[1];
    }
    
    // Edit these grids to set up your own environment. Be aware that the robot starts at [1][5] and the goal is at [8][5].
    std::vector<std::vector<int>> simple_grid = {
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // x=0 (0-8m)
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // x=1 (8-16m)  Start at [1][5]
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // x=2 (16-24m)
        {0, 0, 1, 1, 1, 1, 0, 0, 0, 0},  // x=3 (24-32m)
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // x=4 (32-40m)
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // x=5 (40-48m)
        {0, 0, 0, 0, 0, 1, 1, 1, 1, 1},  // x=6 (48-56m)
        {0, 0, 1, 0, 0, 0, 0, 0, 0, 0},  // x=7 (56-64m)
        {0, 0, 1, 0, 0, 0, 0, 0, 0, 0},  // x=8 (64-72m) Goal at [8][5]
        {0, 0, 1, 0, 0, 0, 0, 0, 0, 0}   // x=9 (72-80m)
    };
    
    std::vector<std::vector<int>> complex_grid = {
        {1, 0, 1, 0, 0, 0, 0, 0, 1, 0},  // x=0 (0-8m)
        {1, 0, 0, 0, 1, 0, 0, 0, 1, 0},  // x=1 (8-16m)  Start at [1][5]
        {1, 0, 1, 1, 1, 1, 1, 1, 1, 0},  // x=2 (16-24m)
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // x=3 (24-32m)
        {0, 0, 1, 0, 1, 1, 0, 0, 0, 0},  // x=4 (32-40m)
        {0, 1, 1, 1, 0, 1, 0, 1, 0, 1},  // x=5 (40-48m)
        {0, 0, 1, 1, 0, 1, 0, 1, 0, 0},  // x=6 (48-56m)
        {0, 0, 1, 0, 1, 1, 1, 1, 0, 1},  // x=7 (56-64m)
        {0, 0, 1, 0, 0, 0, 0, 0, 0, 0},  // x=8 (64-72m) Goal at [8][5]
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}   // x=9 (72-80m)
    };
    
    // Load the appropriate grid
    if (scenario == "simple") {
        std::cout << "Loading complex grid from 10x10 grid..." << std::endl;
        grid.loadFrom10x10Grid(simple_grid);
    } else {
        std::cout << "Loading simple lot grid from 10x10 grid..." << std::endl;
        grid.loadFrom10x10Grid(complex_grid);
    }
    
    // Robot shape (car-like)
    RobotShape robot;
    robot.length = 4.0;  // meters
    robot.width = 2.0;   // meters
    robot.rear_axle_to_center = 1.0;  // meters
    
    std::cout << "Robot dimensions: " << robot.length << "m x " << robot.width << "m" << std::endl;
    
    // State configuration
    StateConfig state_config;
    state_config.xy_resolution = 1.0;  // m
    state_config.yaw_resolution = M_PI / 18;  // degrees
    state_config.curvature_resolution = 0.2;  // rad/m
    state_config.max_curvature = 0.35;  // max curvature
    
    std::cout << "State discretization: xy=" << state_config.xy_resolution 
              << "m, yaw=" << (state_config.yaw_resolution * 180 / M_PI) 
              << "deg, curvature=" << state_config.curvature_resolution << " rad/m" << std::endl;
    
    // Safety margin for added clearance
    double safety_margin = 0.3;  // m
    std::cout << "Safety margin: " << safety_margin << "m" << std::endl;
    
    // Create planner
    HybridAStar planner(grid, robot, state_config, safety_margin);
    
    // Define start and goal states
    State start, goal;
    
    // Start and goal positions
    if (scenario == "complex") {
        // Complex grid setup
        start = State(12.0, 44.0, 0.0, 0.0,  // x, y, yaw, curvature
                     state_config.xy_resolution, state_config.yaw_resolution, state_config.curvature_resolution);
        goal = State(70.0, 44.0, 0.0, 0.0,
                    state_config.xy_resolution, state_config.yaw_resolution, state_config.curvature_resolution);
    } else {
        // Simple grid setup
        start = State(10.0, 40.0, 0.0, 0.0,
                     state_config.xy_resolution, state_config.yaw_resolution, state_config.curvature_resolution);
        goal = State(70.0, 40.0, 0.0, 0.0,
                    state_config.xy_resolution, state_config.yaw_resolution, state_config.curvature_resolution);
    }
    
    std::cout << std::endl;
    std::cout << "Planning from (" << start.x << ", " << start.y << ", " << start.yaw 
              << ") to (" << goal.x << ", " << goal.y << ", " << goal.yaw << ")" << std::endl;
    std::cout << std::endl;
    
    // Plan
    PlanResult result = planner.plan(start, goal, 30000.0);  // 30 second timeout
    
    // Path smoothing and interpolation
    if (result.success && result.path.size() > 2) {
        std::cout << std::endl;
        std::cout << "Starting path smoothing and interpolation" << std::endl;
        
        PathSmoother smoother(grid, robot);
        std::vector<State> original_path = result.path;
        
        // A* ends when tolerance is satisfied, but I want exact goal for smoothing
        original_path.back() = goal;
        
        // Apply smoothing with interpolation
        result.path = smoother.smoothAndInterpolate(original_path, 50, 0.5);
        
        std::cout << "Original: " << original_path.size() << " → Smoothed: " << result.path.size() << " waypoints" << std::endl;
    }
    
    // Print results
    std::cout << std::endl;
    std::cout << "PATH PLANNING RESULTS" << std::endl;
    std::cout << "Success: " << (result.success ? "YES" : "NO") << std::endl;
    std::cout << "Nodes expanded: " << result.nodes_expanded << std::endl;
    std::cout << "Planning time: " << result.planning_time_ms << " ms" << std::endl;
    
    if (result.success) {
        std::cout << "Path length: " << result.path.size() << " waypoints" << std::endl;
        
        // Compute path distance
        double path_distance = 0.0;
        for (size_t i = 1; i < result.path.size(); ++i) {
            double dx = result.path[i].x - result.path[i-1].x;
            double dy = result.path[i].y - result.path[i-1].y;
            path_distance += std::sqrt(dx * dx + dy * dy);
        }
        std::cout << "Path distance: " << path_distance << " meters" << std::endl;
    }
    
    // Write MCAP visualization for Foxglove Studio
    std::cout << std::endl;
    std::cout << "Writing visualization files..." << std::endl;
    
    try {
        MCAPWriter mcap_writer("visualization.mcap");
        mcap_writer.writeVisualization(grid, result, start, goal, robot);
    } catch (const std::exception& e) {
        std::cerr << "Error writing MCAP: " << e.what() << std::endl;
    }
    
    // Also write JSON for Python visualizer
    try {
        MCAPWriter json_writer("visualization.json");
        json_writer.writeVisualizationJSON(grid, result, start, goal, robot);
        std::cout << "JSON written." << std::endl;
        std::cout << "Open the visualization.mcap file in foxglove studio to see the visualization" << std::endl;
        std::cout << "Or run this to see it: python3 visualize.py <visualization.json>" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error writing JSON: " << e.what() << std::endl;
    }
    
    return result.success ? 0 : 1;
}


