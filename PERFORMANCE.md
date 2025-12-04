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

## Optimizations

###Hash map and priority queue:

I used a hash map (`unordered_map`) with a custom hash function (prime multiplication) for fast state lookups, which lets me check if a state was already visited in O(1) time. For the open set in the A* planning, I used a priority queue so I always expand the most promising node first without having to search through everything.

###Precomputed heuristic:

Before running the search, I precompute a heuristic field using Dijkstra's algorithm from the goal. This gives me a better heuristic that accounts for obstacles, helping the search find the goal faster.

###Fast collision detection:

For collision checking, I sample points along the robot's perimeter rather than checking every cell inside the footprint. This cuts down on the number of grid lookups significantly.

###Steer range limiting while maintaining realism:

I limit how much the steering can change between steps, which reduces the branching factor while still producing realistic paths.

###Compiler:

I compile with `-O3 -march=native` to get the best performance.