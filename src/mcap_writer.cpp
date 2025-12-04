#define MCAP_COMPRESSION_NO_LZ4
#define MCAP_COMPRESSION_NO_ZSTD
#define MCAP_IMPLEMENTATION
#include <mcap/writer.hpp>

#include "mcap_writer.h"
#include <nlohmann/json.hpp>
#include <foxglove/SceneUpdate.pb.h>
#include <google/protobuf/descriptor.pb.h>
#include <iostream>
#include <fstream>
#include <cmath>

using json = nlohmann::json;

namespace hybrid_astar {

MCAPWriter::MCAPWriter(const std::string& filename) : filename_(filename) {
}

MCAPWriter::~MCAPWriter() {
}

void MCAPWriter::writeVisualization(const OccupancyGrid& grid,
                                    const PlanResult& result,
                                    const State& start,
                                    const State& goal,
                                    const RobotShape& robot) {
    mcap::McapWriter writer;
    mcap::McapWriterOptions options("");
    options.compression = mcap::Compression::None;
    
    auto status = writer.open(filename_, options);
    if (!status.ok()) {
        std::cerr << "Failed to open MCAP file: " << status.message << std::endl;
        return;
    }
    
    // Get the FileDescriptor for SceneUpdate
    const google::protobuf::FileDescriptor* fd = foxglove::SceneUpdate::descriptor()->file();
    google::protobuf::FileDescriptorSet fdSet;
    
    // Build transitive dependencies
    std::vector<const google::protobuf::FileDescriptor*> toAdd;
    toAdd.push_back(fd);
    while (!toAdd.empty()) {
        const google::protobuf::FileDescriptor* current = toAdd.back();
        toAdd.pop_back();
        current->CopyTo(fdSet.add_file());
        for (int i = 0; i < current->dependency_count(); ++i) {
            toAdd.push_back(current->dependency(i));
        }
    }
    std::string schemaData = fdSet.SerializeAsString();
    
    // Register SceneUpdate schema with protobuf encoding
    mcap::Schema sceneSchema("foxglove.SceneUpdate", "protobuf", schemaData);
    writer.addSchema(sceneSchema);
    
    mcap::Channel sceneChannel("/scene", "protobuf", sceneSchema.id);
    writer.addChannel(sceneChannel);
    
    uint64_t timestamp = 1000000000ULL;
    uint32_t seq = 0;
    
    auto writeProtoMsg = [&](const mcap::Channel& ch, const google::protobuf::Message& msg, uint64_t ts) {
        std::string serialized = msg.SerializeAsString();
        mcap::Message mcapMsg;
        mcapMsg.channelId = ch.id;
        mcapMsg.sequence = seq++;
        mcapMsg.logTime = ts;
        mcapMsg.publishTime = ts;
        mcapMsg.data = reinterpret_cast<const std::byte*>(serialized.data());
        mcapMsg.dataSize = serialized.size();
        auto writeStatus = writer.write(mcapMsg);
        if (!writeStatus.ok()) {
            std::cerr << "Failed to write protobuf message: " << writeStatus.message << std::endl;
        }
    };
    
    // Create SceneUpdate with all visualization elements
    foxglove::SceneUpdate sceneUpdate;
    
    // Entity 1: Path lines colored by direction (green=forward, orange=reverse)
    if (result.success && !result.path.empty()) {
        // Create separate line strips for forward and reverse segments
        foxglove::SceneEntity* forwardPathEntity = sceneUpdate.add_entities();
        forwardPathEntity->mutable_timestamp()->set_sec(timestamp / 1000000000);
        forwardPathEntity->mutable_timestamp()->set_nsec(timestamp % 1000000000);
        forwardPathEntity->set_frame_id("map");
        forwardPathEntity->set_id("path_forward");
        forwardPathEntity->mutable_lifetime()->set_sec(0);
        forwardPathEntity->mutable_lifetime()->set_nsec(0);
        forwardPathEntity->set_frame_locked(false);
        
        foxglove::SceneEntity* reversePathEntity = sceneUpdate.add_entities();
        reversePathEntity->mutable_timestamp()->set_sec(timestamp / 1000000000);
        reversePathEntity->mutable_timestamp()->set_nsec(timestamp % 1000000000);
        reversePathEntity->set_frame_id("map");
        reversePathEntity->set_id("path_reverse");
        reversePathEntity->mutable_lifetime()->set_sec(0);
        reversePathEntity->mutable_lifetime()->set_nsec(0);
        reversePathEntity->set_frame_locked(false);
        
        // Use LINE_LIST to draw individual segments with correct colors
        foxglove::LinePrimitive* forwardLine = forwardPathEntity->add_lines();
        forwardLine->set_type(foxglove::LinePrimitive::LINE_LIST);
        forwardLine->mutable_pose()->mutable_position()->set_x(0);
        forwardLine->mutable_pose()->mutable_position()->set_y(0);
        forwardLine->mutable_pose()->mutable_position()->set_z(0);
        forwardLine->mutable_pose()->mutable_orientation()->set_x(0);
        forwardLine->mutable_pose()->mutable_orientation()->set_y(0);
        forwardLine->mutable_pose()->mutable_orientation()->set_z(0);
        forwardLine->mutable_pose()->mutable_orientation()->set_w(1);
        forwardLine->set_thickness(0.2);
        forwardLine->set_scale_invariant(false);
        forwardLine->mutable_color()->set_r(0.0);
        forwardLine->mutable_color()->set_g(1.0);
        forwardLine->mutable_color()->set_b(0.0);
        forwardLine->mutable_color()->set_a(1.0);
        
        foxglove::LinePrimitive* reverseLine = reversePathEntity->add_lines();
        reverseLine->set_type(foxglove::LinePrimitive::LINE_LIST);
        reverseLine->mutable_pose()->mutable_position()->set_x(0);
        reverseLine->mutable_pose()->mutable_position()->set_y(0);
        reverseLine->mutable_pose()->mutable_position()->set_z(0);
        reverseLine->mutable_pose()->mutable_orientation()->set_x(0);
        reverseLine->mutable_pose()->mutable_orientation()->set_y(0);
        reverseLine->mutable_pose()->mutable_orientation()->set_z(0);
        reverseLine->mutable_pose()->mutable_orientation()->set_w(1);
        reverseLine->set_thickness(0.2);
        reverseLine->set_scale_invariant(false);
        reverseLine->mutable_color()->set_r(1.0);
        reverseLine->mutable_color()->set_g(0.6);
        reverseLine->mutable_color()->set_b(0.0);
        reverseLine->mutable_color()->set_a(1.0);
        
        // Add line segments based on direction
        for (size_t i = 0; i + 1 < result.path.size(); ++i) {
            const auto& curr = result.path[i];
            const auto& next = result.path[i + 1];
            
            foxglove::LinePrimitive* targetLine = (curr.direction >= 0) ? forwardLine : reverseLine;
            
            foxglove::Point3* pt1 = targetLine->add_points();
            pt1->set_x(curr.x);
            pt1->set_y(curr.y);
            pt1->set_z(0.1);
            
            foxglove::Point3* pt2 = targetLine->add_points();
            pt2->set_x(next.x);
            pt2->set_y(next.y);
            pt2->set_z(0.1);
        }
    }
    
    // Entity 2: Robot footprints along path as cubes
    // Note: state (x,y) is rear axle position, but cube needs to be at geometric center
    double center_offset = robot.length / 2.0 - robot.rear_axle_to_center;
    
    if (result.success && !result.path.empty()) {
        int footprint_interval = std::max(1, (int)result.path.size() / 15);
        for (size_t i = 0; i < result.path.size(); i += footprint_interval) {
            const auto& state = result.path[i];
            
            // Compute geometric center of robot
            double center_x = state.x + center_offset * std::cos(state.yaw);
            double center_y = state.y + center_offset * std::sin(state.yaw);
            
            foxglove::SceneEntity* fpEntity = sceneUpdate.add_entities();
            fpEntity->mutable_timestamp()->set_sec(timestamp / 1000000000);
            fpEntity->mutable_timestamp()->set_nsec(timestamp % 1000000000);
            fpEntity->set_frame_id("map");
            fpEntity->set_id("footprint_" + std::to_string(i));
            fpEntity->mutable_lifetime()->set_sec(0);
            fpEntity->mutable_lifetime()->set_nsec(0);
            fpEntity->set_frame_locked(false);
            
            foxglove::CubePrimitive* cube = fpEntity->add_cubes();
            cube->mutable_pose()->mutable_position()->set_x(center_x);
            cube->mutable_pose()->mutable_position()->set_y(center_y);
            cube->mutable_pose()->mutable_position()->set_z(0.2);
            
            // Quaternion from yaw
            double qz = std::sin(state.yaw / 2.0);
            double qw = std::cos(state.yaw / 2.0);
            cube->mutable_pose()->mutable_orientation()->set_x(0);
            cube->mutable_pose()->mutable_orientation()->set_y(0);
            cube->mutable_pose()->mutable_orientation()->set_z(qz);
            cube->mutable_pose()->mutable_orientation()->set_w(qw);
            
            cube->mutable_size()->set_x(robot.length);
            cube->mutable_size()->set_y(robot.width);
            cube->mutable_size()->set_z(0.3);
            
            // Color based on direction
            if (state.direction >= 0) {
                cube->mutable_color()->set_r(0.2);
                cube->mutable_color()->set_g(0.8);
                cube->mutable_color()->set_b(0.2);
                cube->mutable_color()->set_a(0.5);
            } else {
                cube->mutable_color()->set_r(1.0);
                cube->mutable_color()->set_g(0.6);
                cube->mutable_color()->set_b(0.0);
                cube->mutable_color()->set_a(0.5);
            }
        }
    }
    
    // Entity 3: Start footprint (blue)
    {
        // Compute geometric center of robot at start
        double start_center_x = start.x + center_offset * std::cos(start.yaw);
        double start_center_y = start.y + center_offset * std::sin(start.yaw);
        
        foxglove::SceneEntity* startEntity = sceneUpdate.add_entities();
        startEntity->mutable_timestamp()->set_sec(timestamp / 1000000000);
        startEntity->mutable_timestamp()->set_nsec(timestamp % 1000000000);
        startEntity->set_frame_id("map");
        startEntity->set_id("start_footprint");
        startEntity->mutable_lifetime()->set_sec(0);
        startEntity->mutable_lifetime()->set_nsec(0);
        startEntity->set_frame_locked(false);
        
        foxglove::CubePrimitive* cube = startEntity->add_cubes();
        cube->mutable_pose()->mutable_position()->set_x(start_center_x);
        cube->mutable_pose()->mutable_position()->set_y(start_center_y);
        cube->mutable_pose()->mutable_position()->set_z(0.25);
        
        double qz = std::sin(start.yaw / 2.0);
        double qw = std::cos(start.yaw / 2.0);
        cube->mutable_pose()->mutable_orientation()->set_x(0);
        cube->mutable_pose()->mutable_orientation()->set_y(0);
        cube->mutable_pose()->mutable_orientation()->set_z(qz);
        cube->mutable_pose()->mutable_orientation()->set_w(qw);
        
        cube->mutable_size()->set_x(robot.length);
        cube->mutable_size()->set_y(robot.width);
        cube->mutable_size()->set_z(0.35);
        
        cube->mutable_color()->set_r(0.2);
        cube->mutable_color()->set_g(0.2);
        cube->mutable_color()->set_b(1.0);
        cube->mutable_color()->set_a(0.7);
    }
    
    // Entity 4: Goal footprint (red)
    {
        // Compute geometric center of robot at goal
        double goal_center_x = goal.x + center_offset * std::cos(goal.yaw);
        double goal_center_y = goal.y + center_offset * std::sin(goal.yaw);
        
        foxglove::SceneEntity* goalEntity = sceneUpdate.add_entities();
        goalEntity->mutable_timestamp()->set_sec(timestamp / 1000000000);
        goalEntity->mutable_timestamp()->set_nsec(timestamp % 1000000000);
        goalEntity->set_frame_id("map");
        goalEntity->set_id("goal_footprint");
        goalEntity->mutable_lifetime()->set_sec(0);
        goalEntity->mutable_lifetime()->set_nsec(0);
        goalEntity->set_frame_locked(false);
        
        foxglove::CubePrimitive* cube = goalEntity->add_cubes();
        cube->mutable_pose()->mutable_position()->set_x(goal_center_x);
        cube->mutable_pose()->mutable_position()->set_y(goal_center_y);
        cube->mutable_pose()->mutable_position()->set_z(0.25);
        
        double qz = std::sin(goal.yaw / 2.0);
        double qw = std::cos(goal.yaw / 2.0);
        cube->mutable_pose()->mutable_orientation()->set_x(0);
        cube->mutable_pose()->mutable_orientation()->set_y(0);
        cube->mutable_pose()->mutable_orientation()->set_z(qz);
        cube->mutable_pose()->mutable_orientation()->set_w(qw);
        
        cube->mutable_size()->set_x(robot.length);
        cube->mutable_size()->set_y(robot.width);
        cube->mutable_size()->set_z(0.35);
        
        cube->mutable_color()->set_r(1.0);
        cube->mutable_color()->set_g(0.2);
        cube->mutable_color()->set_b(0.2);
        cube->mutable_color()->set_a(0.7);
    }
    
    // Entity 5: Obstacles as cubes
    {
        foxglove::SceneEntity* obsEntity = sceneUpdate.add_entities();
        obsEntity->mutable_timestamp()->set_sec(timestamp / 1000000000);
        obsEntity->mutable_timestamp()->set_nsec(timestamp % 1000000000);
        obsEntity->set_frame_id("map");
        obsEntity->set_id("obstacles");
        obsEntity->mutable_lifetime()->set_sec(0);
        obsEntity->mutable_lifetime()->set_nsec(0);
        obsEntity->set_frame_locked(false);
        
        const auto& gridData = grid.getGrid();
        double resolution = grid.getResolution();
        int downsample = 4;  // Downsample for performance
        
        for (int i = 0; i < grid.getGridWidth(); i += downsample) {
            for (int j = 0; j < grid.getGridHeight(); j += downsample) {
                // Check if any cell in block is occupied
                bool occupied = false;
                for (int di = 0; di < downsample && !occupied; ++di) {
                    for (int dj = 0; dj < downsample && !occupied; ++dj) {
                        if (i + di < grid.getGridWidth() && j + dj < grid.getGridHeight()) {
                            if (gridData[i + di][j + dj]) occupied = true;
                        }
                    }
                }
                
                if (occupied) {
                double x, y;
                    grid.gridToWorld(i + downsample/2, j + downsample/2, x, y);
                    
                    double cubeSize = resolution * downsample;
                    double halfSize = cubeSize / 2.0;
                    double domainW = grid.getWidth();
                    double domainH = grid.getHeight();
                    
                    // Clamp cube position to stay within domain
                    double clampedX = std::max(halfSize, std::min(x, domainW - halfSize));
                    double clampedY = std::max(halfSize, std::min(y, domainH - halfSize));
                    
                    // Adjust size if at edge
                    double sizeX = cubeSize;
                    double sizeY = cubeSize;
                    if (x < halfSize) sizeX = x * 2;
                    if (y < halfSize) sizeY = y * 2;
                    if (x > domainW - halfSize) sizeX = (domainW - x) * 2;
                    if (y > domainH - halfSize) sizeY = (domainH - y) * 2;
                    
                    foxglove::CubePrimitive* cube = obsEntity->add_cubes();
                    cube->mutable_pose()->mutable_position()->set_x(clampedX);
                    cube->mutable_pose()->mutable_position()->set_y(clampedY);
                    cube->mutable_pose()->mutable_position()->set_z(0.5);
                    cube->mutable_pose()->mutable_orientation()->set_x(0);
                    cube->mutable_pose()->mutable_orientation()->set_y(0);
                    cube->mutable_pose()->mutable_orientation()->set_z(0);
                    cube->mutable_pose()->mutable_orientation()->set_w(1);
                    
                    cube->mutable_size()->set_x(std::max(0.1, sizeX));
                    cube->mutable_size()->set_y(std::max(0.1, sizeY));
                    cube->mutable_size()->set_z(1.0);
                    
                    cube->mutable_color()->set_r(0.4);
                    cube->mutable_color()->set_g(0.4);
                    cube->mutable_color()->set_b(0.4);
                    cube->mutable_color()->set_a(0.9);
                }
            }
        }
    }
    
    // Entity 6: Domain border (white rectangle)
    {
        foxglove::SceneEntity* borderEntity = sceneUpdate.add_entities();
        borderEntity->mutable_timestamp()->set_sec(timestamp / 1000000000);
        borderEntity->mutable_timestamp()->set_nsec(timestamp % 1000000000);
        borderEntity->set_frame_id("map");
        borderEntity->set_id("domain_border");
        borderEntity->mutable_lifetime()->set_sec(0);
        borderEntity->mutable_lifetime()->set_nsec(0);
        borderEntity->set_frame_locked(false);
        
        foxglove::LinePrimitive* border = borderEntity->add_lines();
        border->set_type(foxglove::LinePrimitive::LINE_LOOP);
        border->mutable_pose()->mutable_position()->set_x(0);
        border->mutable_pose()->mutable_position()->set_y(0);
        border->mutable_pose()->mutable_position()->set_z(0);
        border->mutable_pose()->mutable_orientation()->set_x(0);
        border->mutable_pose()->mutable_orientation()->set_y(0);
        border->mutable_pose()->mutable_orientation()->set_z(0);
        border->mutable_pose()->mutable_orientation()->set_w(1);
        border->set_thickness(0.1);
        border->set_scale_invariant(false);
        border->mutable_color()->set_r(1.0);
        border->mutable_color()->set_g(1.0);
        border->mutable_color()->set_b(1.0);
        border->mutable_color()->set_a(1.0);
        
        double w = grid.getWidth();
        double h = grid.getHeight();
        
        foxglove::Point3* p1 = border->add_points();
        p1->set_x(0); p1->set_y(0); p1->set_z(0.05);
        
        foxglove::Point3* p2 = border->add_points();
        p2->set_x(w); p2->set_y(0); p2->set_z(0.05);
        
        foxglove::Point3* p3 = border->add_points();
        p3->set_x(w); p3->set_y(h); p3->set_z(0.05);
        
        foxglove::Point3* p4 = border->add_points();
        p4->set_x(0); p4->set_y(h); p4->set_z(0.05);
    }
    
    // Entity 7: Explored states (as small spheres)
    if (!result.explored_states.empty()) {
        foxglove::SceneEntity* exploredEntity = sceneUpdate.add_entities();
        exploredEntity->mutable_timestamp()->set_sec(timestamp / 1000000000);
        exploredEntity->mutable_timestamp()->set_nsec(timestamp % 1000000000);
        exploredEntity->set_frame_id("map");
        exploredEntity->set_id("explored_states");
        exploredEntity->mutable_lifetime()->set_sec(0);
        exploredEntity->mutable_lifetime()->set_nsec(0);
        exploredEntity->set_frame_locked(false);
        
        // Sample explored states for performance
        int sample_interval = std::max(1, (int)result.explored_states.size() / 5000);
        
        for (size_t i = 0; i < result.explored_states.size(); i += sample_interval) {
            const auto& state = result.explored_states[i];
            
            foxglove::SpherePrimitive* sphere = exploredEntity->add_spheres();
            sphere->mutable_pose()->mutable_position()->set_x(state.x);
            sphere->mutable_pose()->mutable_position()->set_y(state.y);
            sphere->mutable_pose()->mutable_position()->set_z(0.02);
            sphere->mutable_pose()->mutable_orientation()->set_x(0);
            sphere->mutable_pose()->mutable_orientation()->set_y(0);
            sphere->mutable_pose()->mutable_orientation()->set_z(0);
            sphere->mutable_pose()->mutable_orientation()->set_w(1);
            
            sphere->mutable_size()->set_x(0.3);
            sphere->mutable_size()->set_y(0.3);
            sphere->mutable_size()->set_z(0.1);
            
            sphere->mutable_color()->set_r(0.5);
            sphere->mutable_color()->set_g(0.8);
            sphere->mutable_color()->set_b(1.0);
            sphere->mutable_color()->set_a(0.3);
        }
        
        std::cout << "Added " << result.explored_states.size() / sample_interval 
                  << " explored state markers (sampled from " << result.explored_states.size() << ")" << std::endl;
    }
    
    // Entity 8: Collision states (red spheres)
    if (!result.collision_states.empty()) {
        foxglove::SceneEntity* collisionEntity = sceneUpdate.add_entities();
        collisionEntity->mutable_timestamp()->set_sec(timestamp / 1000000000);
        collisionEntity->mutable_timestamp()->set_nsec(timestamp % 1000000000);
        collisionEntity->set_frame_id("map");
        collisionEntity->set_id("collision_states");
        collisionEntity->mutable_lifetime()->set_sec(0);
        collisionEntity->mutable_lifetime()->set_nsec(0);
        collisionEntity->set_frame_locked(false);
        
        // Sample collision states for performance
        int collision_sample = std::max(1, (int)result.collision_states.size() / 500);
        int collision_count = 0;
        
        for (size_t i = 0; i < result.collision_states.size(); i += collision_sample) {
            const auto& state = result.collision_states[i];
            
            foxglove::SpherePrimitive* sphere = collisionEntity->add_spheres();
            sphere->mutable_pose()->mutable_position()->set_x(state.x);
            sphere->mutable_pose()->mutable_position()->set_y(state.y);
            sphere->mutable_pose()->mutable_position()->set_z(0.15);
            sphere->mutable_pose()->mutable_orientation()->set_x(0);
            sphere->mutable_pose()->mutable_orientation()->set_y(0);
            sphere->mutable_pose()->mutable_orientation()->set_z(0);
            sphere->mutable_pose()->mutable_orientation()->set_w(1);
            
            sphere->mutable_size()->set_x(0.4);
            sphere->mutable_size()->set_y(0.4);
            sphere->mutable_size()->set_z(0.2);
            
            sphere->mutable_color()->set_r(1.0);
            sphere->mutable_color()->set_g(0.2);
            sphere->mutable_color()->set_b(0.2);
            sphere->mutable_color()->set_a(0.6);
            
            collision_count++;
        }
        
        std::cout << "Added " << collision_count 
                  << " collision markers (sampled from " << result.collision_states.size() << ")" << std::endl;
    }
    
    // Write the SceneUpdate
    writeProtoMsg(sceneChannel, sceneUpdate, timestamp);
    
    writer.close();
    
    std::cout << "MCAP written to " << filename_ << std::endl;
}

std::vector<std::pair<double, double>> MCAPWriter::computeFootprintCorners(
    const State& state, const RobotShape& robot) {
    
    double cos_yaw = std::cos(state.yaw);
    double sin_yaw = std::sin(state.yaw);
    
    double front = robot.length - robot.rear_axle_to_center;
    double rear = -robot.rear_axle_to_center;
    double left = robot.width / 2.0;
    double right = -robot.width / 2.0;
    
    auto transform = [&](double x_robot, double y_robot) -> std::pair<double, double> {
        double x_world = state.x + x_robot * cos_yaw - y_robot * sin_yaw;
        double y_world = state.y + x_robot * sin_yaw + y_robot * cos_yaw;
        return {x_world, y_world};
    };
    
    return {
        transform(front, left),
        transform(front, right),
        transform(rear, right),
        transform(rear, left)
    };
}

void MCAPWriter::writeVisualizationJSON(const OccupancyGrid& grid,
                                        const PlanResult& result,
                                        const State& start,
                                        const State& goal,
                                        const RobotShape& robot) {
    std::ofstream file(filename_);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename_);
    }
    
    json output;
    output["format"] = "hybrid_astar_visualization";
    output["version"] = "1.0";
    output["markers"] = json::array();
    
    // 1. Occupancy grid
    {
        json marker;
        marker["type"] = "occupancy_grid";
        marker["resolution"] = grid.getResolution();
        marker["width"] = grid.getWidth();
        marker["height"] = grid.getHeight();
        marker["cells"] = json::array();
        
        const auto& gridData = grid.getGrid();
        for (int i = 0; i < grid.getGridWidth(); ++i) {
            for (int j = 0; j < grid.getGridHeight(); ++j) {
                if (gridData[i][j]) {
                    double x, y;
                    grid.gridToWorld(i, j, x, y);
                    marker["cells"].push_back({{"x", x}, {"y", y}});
                }
            }
        }
        output["markers"].push_back(marker);
    }
    
    // 2. Explored states
    if (!result.explored_states.empty()) {
        json marker;
        marker["type"] = "explored_states";
        marker["count"] = result.explored_states.size();
        marker["states"] = json::array();
        
        for (const auto& st : result.explored_states) {
            marker["states"].push_back({
                {"x", st.x}, {"y", st.y}, {"yaw", st.yaw}, {"curvature", st.curvature}
            });
        }
        output["markers"].push_back(marker);
    }
    
    // 3. Path
    if (result.success && !result.path.empty()) {
        json marker;
        marker["type"] = "path";
        marker["waypoints"] = json::array();
        
        for (const auto& st : result.path) {
            marker["waypoints"].push_back({
                {"x", st.x}, {"y", st.y}, {"yaw", st.yaw}, 
                {"curvature", st.curvature}, {"direction", st.direction}
            });
        }
        output["markers"].push_back(marker);
    }
    
    // 4. Robot footprints along path
    if (result.success && !result.path.empty()) {
        int step = std::max(1, static_cast<int>(result.path.size() / 25));
        for (size_t i = 0; i < result.path.size(); i += step) {
            auto corners = computeFootprintCorners(result.path[i], robot);
            json marker;
            marker["type"] = "robot_footprint";
            marker["color"] = "path";
            marker["direction"] = result.path[i].direction;
            marker["center"] = {{"x", result.path[i].x}, {"y", result.path[i].y}};
            marker["yaw"] = result.path[i].yaw;
            marker["corners"] = json::array();
            for (const auto& c : corners) {
                marker["corners"].push_back({{"x", c.first}, {"y", c.second}});
            }
            output["markers"].push_back(marker);
        }
    }
    
    // 5. Start
    {
        auto corners = computeFootprintCorners(start, robot);
        json marker;
        marker["type"] = "robot_footprint";
        marker["color"] = "start";
        marker["direction"] = start.direction;
        marker["center"] = {{"x", start.x}, {"y", start.y}};
        marker["yaw"] = start.yaw;
        marker["corners"] = json::array();
        for (const auto& c : corners) {
            marker["corners"].push_back({{"x", c.first}, {"y", c.second}});
        }
        output["markers"].push_back(marker);
    }
    
    // 6. Goal
    {
        auto corners = computeFootprintCorners(goal, robot);
        json marker;
        marker["type"] = "robot_footprint";
        marker["color"] = "goal";
        marker["direction"] = goal.direction;
        marker["center"] = {{"x", goal.x}, {"y", goal.y}};
        marker["yaw"] = goal.yaw;
        marker["corners"] = json::array();
        for (const auto& c : corners) {
            marker["corners"].push_back({{"x", c.first}, {"y", c.second}});
        }
        output["markers"].push_back(marker);
    }
    
    file << output.dump(2);
    file.close();
}

}