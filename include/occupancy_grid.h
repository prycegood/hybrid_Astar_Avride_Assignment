#pragma once

#include <vector>
#include <array>

namespace hybrid_astar {

// Final occupancy grid is calculated from inputted 10x10 grid in main.cpp
 
class OccupancyGrid {
public:
    OccupancyGrid(double width, double height, double resolution);
    
    // Loading environment from 10x10 grid
    void loadFrom10x10Grid(const std::vector<std::vector<int>>& pattern);
    
    // Returning true if cell is occupied or out of bounds
    bool isOccupied(double x, double y) const;
    
    // Converting world coordinates to grid indices
    void worldToGrid(double x, double y, int& grid_x, int& grid_y) const;
    
    // Converting grid indices to world coordinates (cell center)
    void gridToWorld(int grid_x, int grid_y, double& x, double& y) const;
    
    // Getters
    double getWidth() const { return width_; }
    double getHeight() const { return height_; }
    double getResolution() const { return resolution_; }
    int getGridWidth() const { return grid_width_; }
    int getGridHeight() const { return grid_height_; }
    
    const std::vector<std::vector<bool>>& getGrid() const { return grid_; }
    
private:
    double width_;
    double height_;
    double resolution_;
    int grid_width_;
    int grid_height_;
    
    std::vector<std::vector<bool>> grid_; // Final occupancy grid
};

} 