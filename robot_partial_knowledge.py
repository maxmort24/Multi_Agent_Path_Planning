"""
-------------------------------------------------------
[updated robot class]
-------------------------------------------------------
Author:  Rameen Amin, Zayd Syed, Max Mortensen
ID:  169068460, 169029110, 169065545
Email: amin8460@mylaurier.ca, syed9110@mylaurier.ca, mort5545@mylaurier.ca
__updated__ = "2025-07-11", 2025-07-18
-------------------------------------------------------
"""

from search_algorithms import a_star_search, manhattan_distance


class Robot:
    robot_positions = {}  # Class-level dict: {robot_id: (x, y)}

    def __init__(self, robot_id, start_pos, grid_dimensions, goal_pos, full_map, sensor_radius=1):

        self.id = robot_id
        self.start_row, self.start_col = start_pos
        self.goal_row, self.goal_col = goal_pos
        self.grid_rows, self.grid_cols = grid_dimensions

        self.full_map = full_map  # actual map (used only for sensing)
        self.sensor_radius = sensor_radius

        self.position = start_pos
        self.pos_x, self.pos_y = start_pos

        self.record = []  # stores (symbol, x, y)
        self.local_map = [
            ['?' for _ in range(self.grid_cols)] for _ in range(self.grid_rows)]
        
        self.local_map[self.goal_row][self.goal_col] = 'G'
        self.record.append(('G', self.goal_row, self.goal_col))
        
        self.path = []
        self.sense_environment()

    def get_symbol_at(self, grid, row, col):
        if 0 <= row < self.grid_rows and 0 <= col < self.grid_cols:
            return grid[row][col]
        return '?'  # out of bounds is unknown
    
    #Used to help update paths if the bot is stuck
    def replan(self, new_goal):
        self.goal_row, self.goal_col = new_goal
        self.plan_path()

    def sense_environment(self):
        """Scan local area and update local map."""
        cx, cy = self.pos_x, self.pos_y
        for dx in range(-self.sensor_radius, self.sensor_radius + 1):
            for dy in range(-self.sensor_radius, self.sensor_radius + 1):
                nx, ny = cx + dx, cy + dy
                symbol = self.get_symbol_at(self.full_map, nx, ny)
                if 0 <= nx < self.grid_rows and 0 <= ny < self.grid_cols:
                    self.local_map[nx][ny] = symbol
                    record = (symbol, nx, ny)
                    if record not in self.record:
                        self.record.append(record)


    #Updated to stop obstacle phasing(TO add comments)
    def plan_path(self):
            """Plans a path from current position to goal using A* and the local map."""    
            start = self.position
            goal = (self.goal_row, self.goal_col)
            print(f"[Robot {self.id}] Planning path from {start} to {goal}...")
            path = a_star_search(self.local_map, start, goal,
                                heuristic_func=manhattan_distance)
            if path:
                self.path = path[1:]
                print(f"[Robot {self.id}] Path found: {self.path}")
            else:
                print(f"[Robot {self.id}] No path to goal. Trying to explore unknowns...")

                # Look for the nearest '?' tile and go explore it
                unknown_tiles = []
                for r in range(self.grid_rows):
                    for c in range(self.grid_cols):
                        if self.local_map[r][c] == '?':
                            unknown_tiles.append((r, c))

                # Try to path to nearest '?'
                if unknown_tiles:
                    unknown_tiles.sort(key=lambda pos: abs(pos[0] - self.pos_x) + abs(pos[1] - self.pos_y))
                    for target in unknown_tiles:
                        explore_path = a_star_search(self.local_map, start, target,
                                                    heuristic_func=manhattan_distance)
                        if explore_path:
                            self.path = explore_path[1:]
                            print(f"[Robot {self.id}] Exploring toward unknown at {target}")
                            return
                    print(f"[Robot {self.id}] No reachable unknowns found.")
                else:
                    print(f"[Robot {self.id}] Entire map explored. No path.")
                self.path = []
    def move(self):
        """Move one step along the path."""
        if not self.path:
            print(f"[Robot {self.id}] No path to follow.")
            return

        # Remove previous 'R' from map
        prev_x, prev_y = self.pos_x, self.pos_y
        self.local_map[prev_x][prev_y] = '0'

        # Move
        next_pos = self.path.pop(0)

        true_value = self.get_symbol_at(self.full_map, next_pos[0], next_pos[1])
        local_value = self.get_symbol_at(self.local_map, next_pos[0], next_pos[1])
        if true_value == '1':
            print(f"🚨 [Robot {self.id}] PHASED THROUGH OBSTACLE at {next_pos}!")
            print(f"    🔍 Local map saw: '{local_value}' | Full map actually: '1'")

        self.position = next_pos
        self.pos_x, self.pos_y = next_pos
        print(f"[Robot {self.id}] Moved to {self.position}")

        # Update class-level robot position tracking
        Robot.robot_positions[self.id] = (self.pos_x, self.pos_y)

        self.sense_environment()

    def share_knowledge(self):
        """Return known info as a dictionary."""
        return {(x, y): symbol for symbol, x, y in self.record}

    def receive_knowledge(self, shared_data):
        """Update local map with another robot’s knowledge."""
        updated = False
        for (x, y), symbol in shared_data.items():
            if self.local_map[x][y] == '?':
                self.local_map[x][y] = symbol
                self.record.append((symbol, x, y))
                updated = True
        if updated:
            print(f"[Robot {self.id}] Received new info. Replanning path.")
            self.plan_path()

    # def print_local_map(self):
    #     print(f"\n[Robot {self.id}] Local Map View:")
    #     for row in self.local_map:
    #         print(" ".join(str(cell) for cell in row))

    def print_local_map(self):
        "ORIGINAL Print_local_map right above"
        print(f"\n[Robot {self.id}] Local Map View:")

        emoji_map = {
            '0': '\u25FB\uFE0F',     # ◻️ White square = free space
            '1': '\U0001FAA8',       # 🪨 Rock = obstacle
            'R': '\U0001F916',       # 🤖 Robot
            'G': '\U0001F6A9',       # 🚩 Red flag (goal)
            '?': '\u2B1B',           # ⬛ Black square (unknown)
        }

        for row in self.local_map:
            visual_row = [emoji_map.get(str(cell), str(cell)) for cell in row]
            print(" ".join(visual_row))

    def __repr__(self):
        return (f"Robot(id={self.id}, start=({self.start_row},{self.start_col}), "
                f"goal=({self.goal_row},{self.goal_col}), current=({self.pos_x},{self.pos_y}))")