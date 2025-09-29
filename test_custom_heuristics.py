"""
-------------------------------------------------------
Test Harness for Custom Heuristics
-------------------------------------------------------
Author:  Ahmed Nafees
ID:      169053598
Email:   nafe3598@mylaurier.ca
__updated__ = "2025-07-29"
-------------------------------------------------------
This script loads a robot environment and tests only the
custom-made pathfinding algorithms and heuristics.
-------------------------------------------------------
"""
import argparse
from interpret_environment import read_robot_file
from search_algorithms import a_star_search, dynamic_weighted_a_star_search, directional_bias_heuristic
from grid_renderer import visualize_path, print_grid_initial

def run_and_display(algo_name, search_func, grid, start, goal, heuristic, **kwargs):
    """Helper to run a search and display its results."""
    print(f"\n===== Running {algo_name} =====")
    
    path = search_func(grid, start, goal, heuristic_func=heuristic, **kwargs)

    if path:
        print(f"Path Found! Length: {len(path)}, Cost: {len(path) - 1}")
        visualize_path(grid, path, start, goal)
    else:
        print("Search could not find a path.")

def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Run custom pathfinding algorithms."
    )
    parser.add_argument(
        '--file',
        type=str,
        default='robot_room.txt',
        help='Path to the robot environment file (default: robot_room.txt)'
    )
    return parser.parse_args()

def main():
    """
    Main function to run the custom heuristic test harness.
    """
    args = parse_arguments()
    
    try:
        grid_size, robot_starts, goal_pos, grid = read_robot_file(args.file)
    except FileNotFoundError:
        print(f"Error: '{args.file}' not found. Run 'create_environment.py' first.")
        return

    if not robot_starts:
        print("Error: No robots found in the environment file.")
        return
    start_pos = robot_starts[1]

    print("--- Environment Loaded ---")
    print(f"Grid Size: {grid_size}")
    print(f"Robot Start: {start_pos}")
    print(f"Goal: {goal_pos}")
    print_grid_initial(grid)
    print("--------------------------")

    # --- Test 1: A* with Directional Bias Heuristic ---
    run_and_display("A* with Directional Bias", a_star_search, grid, start_pos, goal_pos, directional_bias_heuristic)
    
    # --- Test 2: Dynamic Weighted A* Search Algorithm ---
    print(f"\n===== Running Dynamic Weighted A* Search =====")
    path = dynamic_weighted_a_star_search(grid, start_pos, goal_pos)
    if path:
        print(f"Path Found! Length: {len(path)}, Cost: {len(path) - 1}")
        visualize_path(grid, path, start_pos, goal_pos)
    else:
        print("Search could not find a path.")

if __name__ == "__main__":
    main()