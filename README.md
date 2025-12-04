# Hybrid A* Path Planner

Path planning for autonomous vehicles using Hybrid A* with path smoothing.

## Dependencies

### macOS
```bash
brew install cmake eigen protobuf abseil
pip3 install matplotlib numpy
```

### Ubuntu/Debian
```bash
sudo apt install cmake libeigen3-dev libprotobuf-dev protobuf-compiler libabsl-dev
pip3 install matplotlib numpy
```

## Build

```bash
cd build
cmake ..
make -j8
```

## Run

```bash
# From build directory
./hybrid_astar                # Parking lot scenario
./hybrid_astar maze           # Maze scenario
```

## Visualize

**Python (matplotlib):**
```bash
cd ..
python3 visualize.py build/visualization.json
```

**Foxglove Studio:**
Open `build/visualization.mcap` in [Foxglove Studio](https://foxglove.dev/studio)

