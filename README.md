# Hybrid A* Path Planner Test Assignment

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
./hybrid_astar simple            
./hybrid_astar complex
```

## Visualize

**I used Foxglove Studio so I linked for easy access**
Open `build/visualization.mcap` in [Foxglove Studio](https://foxglove.dev/studio)

**Python (matplotlib):**
```bash
cd ..
python3 visualize.py build/visualization.json
```

