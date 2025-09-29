"""
-------------------------------------------------------
Pathfinding Algorithms and Supporting Structures
-------------------------------------------------------
Author:       Ahmed Nafees
ID:           169053598
Email:        nafe3598@mylaurier.ca
__updated__ = "2025-07-06"
-------------------------------------------------------
This file contains the core logic for the search algorithms
required for the project.
-------------------------------------------------------
"""
import heapq
import math
# =======================================================
# SECTION 1: CORE DATA STRUCTURES & HEURISTICS
# =======================================================
class Node:
    """
    A node class for A* Pathfinding. Each node represents a position on the grid.
    """
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position  # (row, col)

        self.g = 0  # Cost from start to current node
        self.h = 0  # Heuristic cost from current node to end
        self.f = 0  # Total cost (g + h)
        # SMA*
        self.depth = parent.depth + 1 if parent else 0
        self.children = []
        
    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f
    
    def __hash__(self):
        return hash(self.position)

    def __repr__(self):
        return f"Node(pos={self.position}, f={self.f})"

def manhattan_distance(node_a, node_b):
    """
    Calculates the Manhattan distance (L1 norm) between two nodes.
    """
    return abs(node_a.position[0] - node_b.position[0]) + abs(node_a.position[1] - node_b.position[1])

def euclidean_distance(node_a, node_b):
    """
    Calculates the Euclidean distance (L2 norm) between two nodes.
    """
    return math.sqrt((node_a.position[0] - node_b.position[0])**2 + (node_a.position[1] - node_b.position[1])**2)

def _reconstruct_path(end_node):
    """Helper function to reconstruct the path from the end node."""
    path = []
    current = end_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]  # Return reversed path
    
def relaxed_derived_heuristic(node_a, node_b):
    """
    The relaxed-problem derived heuristic that uses optimal path 
    lengths from an obstacle-free map. 
    
    Equivalent to running A* on an obstacle-free map. 
    
    Angela De Oliveira - 169039719
    """
    rows = max(node_a.position[0], node_b.position[0]) + 1
    cols = max(node_a.position[1], node_b.position[1]) + 1
    t_grid = [['0' for _ in range(cols)] for _ in range(rows)]
    
    path = a_star_search(t_grid, node_a.position, node_b.position, manhattan_distance)
    if path is None:
        return float('inf')  # no path is found
    return len(path) - 1  # number of steps

def learned_heuristic(node_a, node_b, grid, radius=3, w1=3, w2=1):
    """
    Learned heuristic: A linear combination of weighted obstacle density
    and goal distance.

    Angela De Oliveira - 169039719
    """
    rows, cols = len(grid), len(grid[0])
    x, y = node_a.position
    obstacle_count = 0

    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            nx, ny = x + dx, y + dy
            if 0 <= nx < rows and 0 <= ny < cols:
                if grid[nx][ny] == '1':
                    obstacle_count += 1

    distance = manhattan_distance(node_a, node_b)

    return (w1 * obstacle_count) + (w2 * distance)

# =======================================================
# SECTION 2: COMPLETED SEARCH ALGORITHMS
# =======================================================

def a_star_search(grid, start_pos, end_pos, heuristic_func):
    """
    Performs A* search to find the shortest path from a start to an end node.
    """
    start_node = Node(None, start_pos)
    end_node = Node(None, end_pos)

    open_list = []
    closed_set = set()

    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.position in closed_set:
            continue
        closed_set.add(current_node.position)

        if current_node == end_node:
            return _reconstruct_path(current_node)

        for move in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            node_position = (current_node.position[0] + move[0], current_node.position[1] + move[1])

            if not (0 <= node_position[0] < len(grid) and 0 <= node_position[1] < len(grid[0])) or \
               grid[node_position[0]][node_position[1]] == '1' or \
               node_position in closed_set:
                continue

            neighbor = Node(current_node, node_position)
            neighbor.g = current_node.g + 1
            neighbor.h = heuristic_func(neighbor, end_node)
            neighbor.f = neighbor.g + neighbor.h
            
            heapq.heappush(open_list, neighbor)

    return None

def greedy_bfs_search(grid, start_pos, end_pos, heuristic_func):
    """
    Performs Greedy Best-First Search
    """
    start_node = Node(None, start_pos)
    end_node = Node(None, end_pos)
    
    start_node.h = heuristic_func(start_node, end_node)
    start_node.f = start_node.h # In Greedy BFS, f = h

    open_list = []
    closed_set = set()

    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.position in closed_set:
            continue
        closed_set.add(current_node.position)

        if current_node == end_node:
            return _reconstruct_path(current_node)

        for move in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            node_position = (current_node.position[0] + move[0], current_node.position[1] + move[1])

            if not (0 <= node_position[0] < len(grid) and 0 <= node_position[1] < len(grid[0])) or \
               grid[node_position[0]][node_position[1]] == '1' or \
               node_position in closed_set:
                continue

            neighbor = Node(current_node, node_position)
            neighbor.h = heuristic_func(neighbor, end_node)
            neighbor.f = neighbor.h  # Key difference: f = h
            
            heapq.heappush(open_list, neighbor)
            
    return None

def weighted_a_star_search(grid, start_pos, end_pos, heuristic_func, weight=1.5):
    """
    Performs Weighted A* search. A weight > 1 makes the search greedier.
    """
    start_node = Node(None, start_pos)
    end_node = Node(None, end_pos)

    open_list = []
    closed_set = set()

    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.position in closed_set:
            continue
        closed_set.add(current_node.position)

        if current_node == end_node:
            return _reconstruct_path(current_node)

        for move in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            node_position = (current_node.position[0] + move[0], current_node.position[1] + move[1])

            if not (0 <= node_position[0] < len(grid) and 0 <= node_position[1] < len(grid[0])) or \
               grid[node_position[0]][node_position[1]] == '1' or \
               node_position in closed_set:
                continue

            neighbor = Node(current_node, node_position)
            neighbor.g = current_node.g + 1
            neighbor.h = heuristic_func(neighbor, end_node)
            neighbor.f = neighbor.g + (neighbor.h * weight) # Key difference: weighted heuristic
            
            heapq.heappush(open_list, neighbor)

    return None

def sma_star_search(grid, start_pos, end_pos, heuristic_func, max_nodes=50):
    """
    Simplified Memory-Bounded A* (SMA*) search implementation.
    
    Navina Thayaruban - 169069359
    Angela De Oliveira - 169039719
    """
    
    start_node = Node(None, start_pos)
    end_node = Node(None, end_pos)
    start_node.h = heuristic_func(start_node, end_node)
    start_node.f = start_node.h

    open_list = []
    closed_set = set()

    heapq.heappush(open_list, start_node)

    while open_list: # pruning
        if len(open_list) > max_nodes:
            worst = max(open_list, key=lambda n: (n.f, -n.depth))
            open_list.remove(worst)
            heapq.heapify(open_list)

            if worst.parent:
                siblings = [c for c in worst.parent.children if c != worst]
                if siblings:
                    best_f = min(child.f for child in siblings)
                    worst.parent.f = max(worst.parent.f, best_f)

        current_node = heapq.heappop(open_list)

        if current_node.position in closed_set:
            continue
        closed_set.add(current_node.position)

        if current_node.position == end_node.position:
            return _reconstruct_path(current_node)

        for move in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            row, col = current_node.position[0] + move[0], current_node.position[1] + move[1]

            if not (0 <= row < len(grid) and 0 <= col < len(grid[0])) or \
               grid[row][col] == '1':
                continue

            position = (row, col)
            if position in closed_set:
                continue

            neighbor = Node(current_node, position)
            neighbor.g = current_node.g + 1
            neighbor.h = heuristic_func(neighbor, end_node)
            neighbor.f = neighbor.g + neighbor.h
            neighbor.depth = current_node.depth + 1

            current_node.children.append(neighbor)
            heapq.heappush(open_list, neighbor)

    return None # if path not found

# =======================================================
# SECTION 3: ADVANCED HEURISTICS & ALGORITHMS
# =======================================================

def dynamic_weighted_a_star_search(grid, start_pos, end_pos):
    """
    An A* search variant that uses a dynamic weight for its heuristic.
    The weight decreases as the robot gets closer to the goal.
    """
    start_node = Node(None, start_pos)
    end_node = Node(None, end_pos)
    
    max_distance = abs(start_pos[0] - end_pos[0]) + abs(start_pos[1] - end_pos[1])

    open_list = []
    closed_set = set()

    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.position in closed_set:
            continue
        closed_set.add(current_node.position)

        if current_node == end_node:
            return _reconstruct_path(current_node)

        for move in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            node_position = (current_node.position[0] + move[0], current_node.position[1] + move[1])

            if not (0 <= node_position[0] < len(grid) and 0 <= node_position[1] < len(grid[0])) or \
               grid[node_position[0]][node_position[1]] == '1' or \
               node_position in closed_set:
                continue

            neighbor = Node(current_node, node_position)
            
            h = manhattan_distance(neighbor, end_node)
            remaining_ratio = h / max_distance if max_distance != 0 else 1
            w = 1 + remaining_ratio # Weight reduces as goal gets closer
            
            neighbor.g = current_node.g + 1
            neighbor.h = h # Store the pure heuristic
            neighbor.f = neighbor.g + w * neighbor.h # Use the dynamic weight for f-cost
            
            heapq.heappush(open_list, neighbor)

    return None

def directional_bias_heuristic(node_a, node_b):
    """
    A heuristic that adds a small penalty for changing direction.
    """
    h = manhattan_distance(node_a, node_b)
    
    # Penalize for changing direction
    if node_a.parent and node_a.parent.parent:
        p1 = node_a.parent.parent.position
        p2 = node_a.parent.position
        p3 = node_a.position
        
        # Direction from grandparent to parent
        dir1 = (p2[0] - p1[0], p2[1] - p1[1])
        # Direction from parent to current
        dir2 = (p3[0] - p2[0], p3[1] - p2[1])
        
        if dir1 != dir2:
            h += 1 # Add penalty for turning
    return h