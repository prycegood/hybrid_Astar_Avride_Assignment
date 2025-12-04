#include "heuristic.h"
#include <queue>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace hybrid_astar {

Heuristic::Heuristic(const OccupancyGrid& grid, const State& goal)
    : grid_(grid), goal_(goal), heuristic_computed_(false),
      obstacle_weight_(20.0), robot_clearance_(5.5) {  // These weights seem to work well
    
    // Initialize distance fields
    int width = grid.getGridWidth();
    int height = grid.getGridHeight();
    
    goal_distance_field_.resize(width, std::vector<double>(height, std::numeric_limits<double>::infinity()));
    obstacle_distance_field_.resize(width, std::vector<double>(height, std::numeric_limits<double>::infinity()));
    voronoi_field_.resize(width, std::vector<double>(height, std::numeric_limits<double>::infinity()));
}

double Heuristic::compute(const State& current) const {
    if (!heuristic_computed_) {
        // Fallback to simple heuristic if not precomputed
        return euclideanDistance(current, goal_);
    }
    
    // Use Voronoi field heuristic
    return getHolonomicDistance(current.x, current.y);
}

void Heuristic::precomputeHolonomicHeuristic() {
    std::cout << "Computing Voronoi field heuristic..." << std::endl;
    
    //Compute obstacle distance transform
    computeObstacleDistanceTransform();
    
    //Compute goal distance field
    computeGoalDistanceField();
    
    //Combine into Voronoi field
    computeVoronoiField();
    
    heuristic_computed_ = true;
    std::cout << "Voronoi field computation complete." << std::endl;
}

double Heuristic::getHolonomicDistance(double x, double y) const {
    if (!heuristic_computed_) {
        return euclideanDistance(State(x, y, 0, 0, 1.0, 1.0, 1.0), goal_);
    }
    
    int grid_x, grid_y;
    grid_.worldToGrid(x, y, grid_x, grid_y);
    
    if (grid_x < 0 || grid_x >= static_cast<int>(voronoi_field_.size()) || 
        grid_y < 0 || grid_y >= static_cast<int>(voronoi_field_[0].size())) {
        return std::numeric_limits<double>::infinity();
    }
    
    return voronoi_field_[grid_x][grid_y];
}

double Heuristic::euclideanDistance(const State& a, const State& b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

double Heuristic::nonholonomicDistance(const State& a, const State& b, double max_curvature) {
    // Simplified Reeds-Shepp distance estimate
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double straight_dist = std::sqrt(dx * dx + dy * dy);
    
    // Minimum turning radius
    double min_radius = 1.0 / max_curvature;
    
    // Angular difference
    double angle_diff = std::abs(State::normalizeAngle(b.yaw - a.yaw));
    
    // Estimate: straight distance + turning cost
    double turning_cost = min_radius * angle_diff;
    
    return straight_dist + turning_cost;
}

// Populating the obstacle distance field
void Heuristic::computeObstacleDistanceTransform() {
    // Compute distance to nearest obstacle for each cell using brushfire algorithm
    int width = grid_.getGridWidth();
    int height = grid_.getGridHeight();
    
    std::queue<std::pair<int, int>> queue;
    
    // Initialize: obstacles have distance 0, free space has infinity
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            double wx, wy;
            grid_.gridToWorld(x, y, wx, wy);
            
            if (grid_.isOccupied(wx, wy)) {
                obstacle_distance_field_[x][y] = 0.0;
                queue.push({x, y});
            } else {
                obstacle_distance_field_[x][y] = std::numeric_limits<double>::infinity();
            }
        }
    }
    
    // Brushfire: propagate distances from obstacles
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const double costs[] = {1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414};
    
    while (!queue.empty()) {
        auto [x, y] = queue.front();
        queue.pop();
        
        double current_dist = obstacle_distance_field_[x][y];
        
        // Expand to neighbors
        for (int i = 0; i < 8; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                continue;
            }
            
            double new_dist = current_dist + costs[i] * grid_.getResolution();
            
            // If the new distance is less than the current distance, update the distance
            if (new_dist < obstacle_distance_field_[nx][ny]) {
                obstacle_distance_field_[nx][ny] = new_dist;
                queue.push({nx, ny});
            }
        }
    }
}

// Populating the goal distance field
void Heuristic::computeGoalDistanceField() {
    // 2D Dijkstra from goal (backward search) considering obstacles
    int width = grid_.getGridWidth();
    int height = grid_.getGridHeight();
    
    // Priority queue: (distance, grid_x, grid_y)
    using QueueElement = std::tuple<double, int, int>;
    std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> pq;
    
    // Start from goal
    int goal_x, goal_y;
    grid_.worldToGrid(goal_.x, goal_.y, goal_x, goal_y);
    
    if (goal_x >= 0 && goal_x < width && goal_y >= 0 && goal_y < height) {
        goal_distance_field_[goal_x][goal_y] = 0.0;
        pq.push(std::make_tuple(0.0, goal_x, goal_y));
    }
    
    // 8-connected grid
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const double costs[] = {1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414};
    
    while (!pq.empty()) {
        auto [dist, x, y] = pq.top();
        pq.pop();
        
        // Skip if already processed with better distance
        if (dist > goal_distance_field_[x][y]) {
            continue;
        }
        
        // Expand neighbors
        for (int i = 0; i < 8; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            
            // Check bounds
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                continue;
            }
            
            // Skip occupied cells
            double wx, wy;
            grid_.gridToWorld(nx, ny, wx, wy);
            if (grid_.isOccupied(wx, wy)) {
                continue;
            }
            
            // Update distance
            double new_dist = dist + costs[i] * grid_.getResolution();
            if (new_dist < goal_distance_field_[nx][ny]) {
                goal_distance_field_[nx][ny] = new_dist;
                pq.push(std::make_tuple(new_dist, nx, ny));
            }
        }
    }
}

// Populating the Voronoi field
void Heuristic::computeVoronoiField() {
    // Voronoi field guides paths toward goal while staying in "safe corridors"
    int width = grid_.getGridWidth();
    int height = grid_.getGridHeight();
    
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            double goal_dist = goal_distance_field_[x][y];
            double obstacle_dist = obstacle_distance_field_[x][y];
            double decay_rate = 5.0;  // Controls how fast penalty decays
            double obstacle_penalty = obstacle_weight_ * std::exp(-decay_rate * obstacle_dist / robot_clearance_);
            //Adds exponential penalty to the goal distance
            voronoi_field_[x][y] = goal_dist + obstacle_penalty;
        }
    }
}

}
