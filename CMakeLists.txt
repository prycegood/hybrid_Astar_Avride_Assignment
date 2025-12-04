cmake_minimum_required(VERSION 3.16)
project(hybrid_astar)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -Wall -Wextra")

# Fetch dependencies
include(FetchContent)

# Set CMake policies to avoid warnings
cmake_policy(SET CMP0135 NEW)

# Fetch MCAP library (header-only) via URL to avoid git-lfs issues
FetchContent_Declare(
    mcap
    URL https://github.com/foxglove/mcap/archive/refs/tags/releases/cpp/v1.4.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(mcap)

# Fetch nlohmann/json for JSON Schema support
FetchContent_Declare(
    json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(json)

# Find Protobuf and Abseil (required by newer protobuf)
find_package(Protobuf REQUIRED)
find_package(absl REQUIRED)

# Local Foxglove proto files
set(PROTO_DIR "${CMAKE_SOURCE_DIR}/proto")
set(PROTO_OUTPUT_DIR "${CMAKE_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${PROTO_OUTPUT_DIR}/foxglove")

# List of proto files to compile (SceneUpdate is self-contained)
set(FOXGLOVE_PROTOS
    ${PROTO_DIR}/foxglove/SceneUpdate.proto
)

# Generate C++ from proto files
set(PROTO_SRCS 
    "${PROTO_OUTPUT_DIR}/foxglove/SceneUpdate.pb.cc"
)
set(PROTO_HDRS 
    "${PROTO_OUTPUT_DIR}/foxglove/SceneUpdate.pb.h"
)

add_custom_command(
    OUTPUT ${PROTO_SRCS} ${PROTO_HDRS}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
        --proto_path=${PROTO_DIR}
        --cpp_out=${PROTO_OUTPUT_DIR}
        ${FOXGLOVE_PROTOS}
    DEPENDS ${FOXGLOVE_PROTOS}
    COMMENT "Generating Foxglove protobuf files"
)

add_custom_target(foxglove_proto_gen DEPENDS ${PROTO_SRCS} ${PROTO_HDRS})

# Find dependencies
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${mcap_SOURCE_DIR}/cpp/mcap/include
    ${PROTO_OUTPUT_DIR}
    ${Protobuf_INCLUDE_DIRS}
)

# Source files
set(SOURCES
    src/main.cpp
    src/state.cpp
    src/occupancy_grid.cpp
    src/collision_checker.cpp
    src/heuristic.cpp
    src/hybrid_astar.cpp
    src/path_smoother.cpp
    src/mcap_writer.cpp
    ${PROTO_SRCS}
)

# Create executable
add_executable(hybrid_astar ${SOURCES})
add_dependencies(hybrid_astar foxglove_proto_gen)

# Link libraries
target_link_libraries(hybrid_astar 
    Eigen3::Eigen 
    nlohmann_json::nlohmann_json
    protobuf::libprotobuf
    absl::log_internal_check_op
    absl::log_internal_message
)

