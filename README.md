### README

# RTS Battle Unit Pathfinding

This project implements a pathfinding algorithm for battle units in a Real-Time Strategy (RTS) game. The algorithm uses the A* search algorithm to find the shortest path from a starting position to a goal position on a randomly generated battlefield map.

## Features

- The battlefield is represented as a 64x64 grid.
- Positions on the battlefield can either be ground terrain (reachable) or elevated terrain (unreachable).
- The A* search algorithm is used to find the shortest path between two points on the battlefield.
- The algorithm considers movement constraints: units can only move horizontally or vertically.

## Prerequisites

- Python 3.x
- numpy

## Installation

1. Ensure you have Python 3.x installed on your machine.
2. Install the required library numpy using pip:

    ```bash
    pip install numpy
    ```

## Usage

1. Clone the repository or download the script.
2. Run the script:

    ```bash
    python pathfinding.py
    ```

## How it Works

1. **Import Libraries**

    ```python
    import heapq
    import numpy as np
    ```

2. **Define Movement Directions**

    The `directions` list defines the possible movement directions for the battle unit: right, left, down, and up.

    ```python
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    ```

3. **Heuristic Function**

    The `heuristic` function calculates the Manhattan distance between two points `a` and `b`.

    ```python
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    ```

4. **Validation Function**

    The `is_valid` function checks if a given position is within the boundaries of the battlefield and if the position is reachable (not elevated terrain).

    ```python
    def is_valid(pos, battlefield):
        x, y = pos
        return 0 <= x < len(battlefield) and 0 <= y < len(battlefield[0]) and battlefield[x][y] != 4
    ```

5. **A* Search Algorithm**

    The `astar` function implements the A* search algorithm. It uses a priority queue (`open_set`) to explore the shortest path from the start position to the goal position. It keeps track of the path using the `came_from` dictionary and calculates the cost of movement using `g_score` and `f_score`.

    ```python
    def astar(start, goal, battlefield):
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]
            
            for direction in directions:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                if not is_valid(neighbor, battlefield):
                    continue
                    
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return None  # No valid path found
    ```

6. **Example Usage**

    The `main` function demonstrates the usage of the A* algorithm on a randomly generated battlefield map. It sets up the battlefield, defines start and goal positions, and runs the algorithm to find the path.

    ```python
    def main():
        battlefield = np.full((64, 64), -1)
        
        np.random.seed(0)
        elevated_positions = np.random.choice(64*64, size=512, replace=False)
        for pos in elevated_positions:
            battlefield[pos // 64][pos % 64] = 4
        
        start = (0, 0)
        goal = (63, 63)
        battlefield[start[0]][start[1]] = 0
        battlefield[goal[0]][goal[1]] = 6
        
        path = astar(start, goal, battlefield)
        if path:
            print("Path found:", path)
        else:
            print("No valid path found.")

    if __name__ == "__main__":
        main()
    ```

The script generates a random battlefield map with 64x64 grid cells, marks some cells as elevated terrain, and uses the A* algorithm to find the shortest path from the start to the goal position.
