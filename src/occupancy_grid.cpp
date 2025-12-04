#include "occupancy_grid.h"
#include <cmath>
#include <iostream>

namespace hybrid_astar {

OccupancyGrid::OccupancyGrid(double width, double height, double resolution)
    : width_(width), height_(height), resolution_(resolution) {
    
    grid_width_ = static_cast<int>(std::ceil(width / resolution));
    grid_height_ = static_cast<int>(std::ceil(height / resolution));
    // Initialize high-res grid as all free
    grid_.resize(grid_width_, std::vector<bool>(grid_height_, false));
}

void OccupancyGrid::loadFrom10x10Grid(const std::vector<std::vector<int>>& pattern) {
    // Validate input
    if (pattern.size() != 10) {
        std::cerr << "Grid must be 10x10" << std::endl;
        return;
    }
    for (size_t i = 0; i < pattern.size(); ++i) {
        if (pattern[i].size() != 10) {
            std::cerr << "Grid must be 10x10" << std::endl;
            return;
        }
    }
    
    // clear grid first
    for (int x = 0; x < grid_width_; ++x) {
        for (int y = 0; y < grid_height_; ++y) {
            grid_[x][y] = false;
        }
    }
    
    // map the 10x10 grid to the actual grid (each coarse cell is 10mx10m)
    double coarse_cell_width = width_ / 10.0;
    double coarse_cell_height = height_ / 10.0;
    
    for (int coarse_x = 0; coarse_x < 10; ++coarse_x) {
        for (int coarse_y = 0; coarse_y < 10; ++coarse_y) {
            if (pattern[coarse_x][coarse_y] == 1) {
                // Cell occupied so fill all fine cells here
                double x_start = coarse_x * coarse_cell_width;
                double x_end = (coarse_x + 1) * coarse_cell_width;
                double y_start = coarse_y * coarse_cell_height;
                double y_end = (coarse_y + 1) * coarse_cell_height;
                
                int grid_x_start = static_cast<int>(x_start / resolution_);
                int grid_x_end = static_cast<int>(std::ceil(x_end / resolution_));
                int grid_y_start = static_cast<int>(y_start / resolution_);
                int grid_y_end = static_cast<int>(std::ceil(y_end / resolution_));
                
                // Filling the cells
                for (int x = grid_x_start; x < grid_x_end && x < grid_width_; ++x) {
                    for (int y = grid_y_start; y < grid_y_end && y < grid_height_; ++y) {
                        grid_[x][y] = true;
                    }
                }
            }
        }
    }
}

bool OccupancyGrid::isOccupied(double x, double y) const {
    int grid_x, grid_y;
    worldToGrid(x, y, grid_x, grid_y);
    
    // Out of bounds is considered occupied
    if (grid_x < 0 || grid_x >= grid_width_ || grid_y < 0 || grid_y >= grid_height_) {
        return true;
    }
    
    return grid_[grid_x][grid_y];
}

void OccupancyGrid::worldToGrid(double x, double y, int& grid_x, int& grid_y) const {
    grid_x = static_cast<int>(std::floor(x / resolution_));
    grid_y = static_cast<int>(std::floor(y / resolution_));
}

void OccupancyGrid::gridToWorld(int grid_x, int grid_y, double& x, double& y) const {
    // Return center of cell
    x = (grid_x + 0.5) * resolution_;
    y = (grid_y + 0.5) * resolution_;
}

}
