from interpret_environment import read_robot_file
from robot_partial_knowledge import Robot
from grid_renderer import print_grid_initial  # optional
import time
import sys
sys.stdout = open("multi_robot_log.txt", "w", encoding="utf-8")


def broadcast_knowledge(robots):
    all_shared_data = {}  # {(x, y): symbol}

    # Aggregate all robot knowledge into one broadcast package
    for robot in robots:
        robot_data = robot.share_knowledge()
        all_shared_data.update(robot_data)

    # Broadcast to all robots
    for robot in robots:
        robot.receive_knowledge(all_shared_data)


def update_all_robot_positions(robots):
    for robot in robots:
        # Clear old robot positions (R)
        for i in range(robot.grid_rows):
            for j in range(robot.grid_cols):
                if robot.local_map[i][j] == 'R':
                    robot.local_map[i][j] = '0'

        # Re-mark current positions of all robots
        for rid, (x, y) in Robot.robot_positions.items():
            robot.local_map[x][y] = 'R'


# Used to help avoid collisions of bots
def get_safe_moves(robot, robots):
    next_pos = robot.next_move  # e.g. (x, y)

    for other in robots:
        if other.id == robot.id:
            continue

        # If other robot is currently on the tile I want
        if other.position == next_pos:
            return False

        # If the other robot is ALSO planning to move into the same tile
        if other.next_move == next_pos:
            # Let the robot with lower ID go first, others wait
            if other.id < robot.id:
                return False

    return True


def run_simulation():
    filepath = "robot_room.txt"
    try:
        grid_size, robot_starts, goal_pos, full_map = read_robot_file(filepath)
    except FileNotFoundError:
        print(
            f"Error: '{filepath}' not found. Run 'create_environment.py' first.")
        return

    print("--- Environment Loaded ---")
    print(f"Grid Size: {grid_size}")
    print(f"Robot Starts: {robot_starts}")
    print(f"Goal: {goal_pos}")
    print_grid_initial(full_map)

    # === Initialize robots ===
    robots = []
    for idx, start_pos in enumerate(robot_starts[:4], start=1):  # max 4 robots
        robot = Robot(
            robot_id=idx,
            start_pos=start_pos,
            goal_pos=goal_pos,
            grid_dimensions=grid_size,
            full_map=full_map
        )
        robots.append(robot)

    # === Plan initial paths ===
    for robot in robots:
        robot.plan_path()

    max_steps = 100  # optional safety cap to prevent infinite loops
    time_step = 0

    while time_step < max_steps:
        print(f"\n--- Time Step {time_step} ---")
        time_step += 1

        # === Phase 1: Determine next moves ===
        next_moves = {}
        for robot in robots:
            if robot.path:
                robot.next_move = robot.path[0]
                next_moves[robot.id] = robot.next_move
            else:
                robot.next_move = None
                next_moves[robot.id] = None

        # === Phase 2: Move if safe ===
        any_robot_moved = False

        for robot in robots:
            if robot.position == (robot.goal_row, robot.goal_col):
                continue  # Already at goal

            move = robot.next_move
            if move is None:
                continue

            conflicting_ids = [rid for rid,
                               m in next_moves.items() if m == move]
            if len(conflicting_ids) > 1:
                if robot.id != min(conflicting_ids):
                    print(f"[Robot {robot.id}] Waiting for priority robot.")
                    robot.wait_count = getattr(robot, "wait_count", 0) + 1
                    if robot.wait_count >= 2:
                        robot.replan((robot.goal_row, robot.goal_col))
                        robot.wait_count = 0
                    continue

            if any(other.position == move for other in robots if other.id != robot.id):
                if move == (robot.goal_row, robot.goal_col):
                    print(
                        f"[Robot {robot.id}] Entering goal even though it's occupied.")
                else:
                    print(
                        f"[Robot {robot.id}] Tile occupied by another robot.")
                    robot.wait_count = getattr(robot, "wait_count", 0) + 1
                    if robot.wait_count >= 2:
                        robot.replan((robot.goal_row, robot.goal_col))
                        robot.wait_count = 0
                    continue

            # Safe to move
            robot.move()
            robot.wait_count = 0
            robot.plan_path()
            any_robot_moved = True

        # Initiate broadcast communication
        broadcast_knowledge(robots)

        # Update robot positions in all maps
        update_all_robot_positions(robots)
        # All robots essentially share a map we are printing the same map n times
        robots[0].print_local_map()

        # Stop early if all robots reached goal
        if all(robot.position == (robot.goal_row, robot.goal_col) for robot in robots):
            print("\n✅ All robots reached the goal!")
            break

        if not any_robot_moved and all(robot.path == [] for robot in robots):
            print("\n❌ No further progress possible. Robots are stuck or no path exists.")
            break

        time.sleep(0.5)  # optional pause to simulate time

    if time_step == max_steps:
        print("Maximum steps used. Simulation terminated ❌.")


if __name__ == "__main__":
    run_simulation()
    sys.stdout.close()