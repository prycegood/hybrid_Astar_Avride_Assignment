#include "state.h"
#include <cmath>

namespace hybrid_astar {

State::State(double x_, double y_, double yaw_, double curv_,
             double xy_resolution, double yaw_resolution, double curv_resolution)
    : x(x_), y(y_), yaw(normalizeAngle(yaw_)), curvature(curv_),
      g(0), h(0), parent(nullptr), direction(1) {
    
    // Discretize continuous values to grid indices so we can do A*
    grid_x = static_cast<int>(std::round(x / xy_resolution));
    grid_y = static_cast<int>(std::round(y / xy_resolution));
    grid_yaw = static_cast<int>(std::round(yaw / yaw_resolution));
    grid_curvature = static_cast<int>(std::round(curvature / curv_resolution));
}

double State::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

size_t State::hash() const {
    //prime number based hash function.
    //includes direction because Reeds-shepp needs it
    size_t h1 = std::hash<int>{}(grid_x);
    size_t h2 = std::hash<int>{}(grid_y);
    size_t h3 = std::hash<int>{}(grid_yaw);
    size_t h4 = std::hash<int>{}(grid_curvature);
    size_t h5 = std::hash<int>{}(direction);
    
    size_t hash_value = 17;
    hash_value = hash_value * 31 + h1;
    hash_value = hash_value * 31 + h2;
    hash_value = hash_value * 31 + h3;
    hash_value = hash_value * 31 + h4;
    hash_value = hash_value * 31 + h5;
    
    return hash_value;
}

}
