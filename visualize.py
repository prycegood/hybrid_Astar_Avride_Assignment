#!/usr/bin/env python3
"""
Visualization script for Hybrid A* planner output

Usage: python3 visualize.py <visualization.json>
"""

import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import sys

def load_visualization(filename):
    """Load visualization JSON file"""
    with open(filename, 'r') as f:
        data = json.load(f)
    return data

def plot_visualization(data):
    """Create visualization plot"""
    fig, ax = plt.subplots(figsize=(12, 12))
    
    # Plot occupancy grid
    print("Plotting occupancy grid...")
    for marker in data['markers']:
        if marker['type'] == 'occupancy_grid':
            resolution = marker['resolution']
            for cell in marker['cells']:
                rect = patches.Rectangle(
                    (cell['x'] - resolution/2, cell['y'] - resolution/2),
                    resolution, resolution,
                    linewidth=0,
                    edgecolor='none',
                    facecolor='gray',
                    alpha=0.7
                )
                ax.add_patch(rect)
            
            # Set plot limits based on grid
            ax.set_xlim(0, marker['width'])
            ax.set_ylim(0, marker['height'])
            print(f"  Grid: {marker['width']}m x {marker['height']}m @ {resolution}m resolution")
            print(f"  Obstacles: {len(marker['cells'])} cells")
    
    # Plot explored states
    print("Plotting explored states...")
    for marker in data['markers']:
        if marker['type'] == 'explored_states':
            states = marker['states']
            if states:
                x_coords = [s['x'] for s in states]
                y_coords = [s['y'] for s in states]
                ax.scatter(x_coords, y_coords, c='lightblue', s=1, alpha=0.3, 
                          label=f'Explored ({len(states)} states)')
                print(f"  Explored states: {len(states)}")
    
    # Plot path
    print("Plotting path...")
    for marker in data['markers']:
        if marker['type'] == 'path':
            waypoints = marker['waypoints']
            if waypoints:
                x_coords = [w['x'] for w in waypoints]
                y_coords = [w['y'] for w in waypoints]
                
                # Color path by direction (green=forward, orange=reverse)
                for i in range(len(waypoints) - 1):
                    color = 'green' if waypoints[i]['direction'] > 0 else 'orange'
                    ax.plot([x_coords[i], x_coords[i+1]], 
                           [y_coords[i], y_coords[i+1]], 
                           color=color, linewidth=2, alpha=0.8)
                
                # Add arrows to show direction
                arrow_spacing = max(1, len(waypoints) // 20)
                for i in range(0, len(waypoints) - 1, arrow_spacing):
                    dx = x_coords[i+1] - x_coords[i]
                    dy = y_coords[i+1] - y_coords[i]
                    ax.arrow(x_coords[i], y_coords[i], dx*0.7, dy*0.7,
                            head_width=0.5, head_length=0.3, 
                            fc='darkgreen' if waypoints[i]['direction'] > 0 else 'darkorange',
                            ec='darkgreen' if waypoints[i]['direction'] > 0 else 'darkorange',
                            alpha=0.6)
                
                print(f"  Path waypoints: {len(waypoints)}")
                
                # Compute path length
                path_length = 0
                for i in range(len(waypoints) - 1):
                    dx = x_coords[i+1] - x_coords[i]
                    dy = y_coords[i+1] - y_coords[i]
                    path_length += np.sqrt(dx**2 + dy**2)
                print(f"  Path length: {path_length:.2f}m")
    
    # Plot robot footprints
    print("Plotting robot footprints...")
    footprint_count = {'start': 0, 'goal': 0, 'path_forward': 0, 'path_reverse': 0}
    for marker in data['markers']:
        if marker['type'] == 'robot_footprint':
            corners = marker['corners']
            color_name = marker['color']
            direction = marker.get('direction', 1)  # Default to forward if not specified
            
            # Map color names to actual colors, considering direction
            if color_name == 'path':
                # Use different colors for forward vs reverse motion
                if direction < 0:
                    color = 'orange'  # Reverse
                    footprint_count['path_reverse'] += 1
                else:
                    color = 'green'   # Forward
                    footprint_count['path_forward'] += 1
            else:
                # Start and goal use fixed colors
                color_map = {
                    'start': 'blue',
                    'goal': 'red',
                }
                color = color_map.get(color_name, 'black')
                footprint_count[color_name] = footprint_count.get(color_name, 0) + 1
            
            # Create polygon
            polygon_points = [(c['x'], c['y']) for c in corners]
            polygon = patches.Polygon(polygon_points, linewidth=1.5, 
                                     edgecolor=color, facecolor=color, alpha=0.3)
            ax.add_patch(polygon)
            
    
    print(f"  Start: {footprint_count.get('start', 0)}")
    print(f"  Goal: {footprint_count.get('goal', 0)}")
    print(f"  Along path (forward): {footprint_count.get('path_forward', 0)}")
    print(f"  Along path (reverse): {footprint_count.get('path_reverse', 0)}")
    
    # Formatting
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (meters)', fontsize=12)
    ax.set_ylabel('Y (meters)', fontsize=12)
    ax.set_title('Hybrid A* Path Planning Visualization', fontsize=14, fontweight='bold')
    
    # Create custom legend
    from matplotlib.lines import Line2D
    legend_elements = [
        patches.Patch(facecolor='gray', alpha=0.7, label='Obstacles'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='lightblue', 
               markersize=8, alpha=0.5, label='Explored states'),
        Line2D([0], [0], color='green', linewidth=2, label='Path (forward)'),
        Line2D([0], [0], color='orange', linewidth=2, label='Path (reverse)'),
        patches.Patch(facecolor='blue', alpha=0.3, edgecolor='blue', label='Start pose'),
        patches.Patch(facecolor='red', alpha=0.3, edgecolor='red', label='Goal pose'),
    ]
    ax.legend(handles=legend_elements, loc='upper left', fontsize=10)
    
    plt.tight_layout()
    
    # Save figure
    output_file = 'visualization.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\nVisualization saved to: {output_file}")
    
    # Show plot
    plt.show()

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 visualize.py <visualization.json>")
        print("Example: python3 visualize.py build/visualization.json")
        sys.exit(1)
    
    filename = sys.argv[1]
    
    print(f"Loading visualization from: {filename}")
    try:
        data = load_visualization(filename)
        print("Successfully loaded visualization data")
        print()
        plot_visualization(data)
    except FileNotFoundError:
        print(f"Error: File not found: {filename}")
        sys.exit(1)
    except json.JSONDecodeError:
        print(f"Error: Invalid JSON format in {filename}")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()


