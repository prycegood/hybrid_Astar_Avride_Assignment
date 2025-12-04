#pragma once

#include "state.h"
#include "occupancy_grid.h"
#include "hybrid_astar.h"
#include <string>
#include <vector>

namespace hybrid_astar {

// MCAP Writer for Foxglove Visualization
class MCAPWriter {
public:
    MCAPWriter(const std::string& filename);
    ~MCAPWriter();

    void writeVisualization(const OccupancyGrid& grid,
                           const PlanResult& result,
                           const State& start,
                           const State& goal,
                           const RobotShape& robot);
    
    void writeVisualizationJSON(const OccupancyGrid& grid,
                                const PlanResult& result,
                                const State& start,
                                const State& goal,
                                const RobotShape& robot);

private:
    std::string filename_;
    
    std::vector<std::pair<double, double>> computeFootprintCorners(
        const State& state, const RobotShape& robot);
};

}
