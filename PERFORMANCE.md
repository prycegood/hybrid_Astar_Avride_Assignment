# Performance Report

## Hardware
- **CPU:** Apple M2 Pro
- **RAM:** 16 GB
- **OS:** macOS

## Results
| Metric | Value |
|--------|-------|
| Nodes expanded | 176,385 |
| Total planning time | 5,172 ms |
| Time per node | 0.029 ms |
| **Time for 100k expansions** | **~2,932 ms** |
| Nodes per second | ~34,100 |

## Optimizations Used

1. **Hash-based state lookup** - `std::unordered_map` with custom hash function for O(1) duplicate state detection

2. **Priority queue** - `std::priority_queue` with custom comparator for efficient open set management

3. **Precomputed heuristics** - 2D Dijkstra distance field computed once before search to guide expansion toward goal

4. **Discretized collision checking** - Sample points along robot perimeter rather than checking entire footprint area

5. **Bounded curvature changes** - Limit steering rate to reduce branching factor while maintaining kinematic feasibility

6. **Compiler optimizations** - Built with `-O3 -march=native` flags for maximum performance