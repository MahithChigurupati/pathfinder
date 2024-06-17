import heapq
import numpy as np

# directions for movement (right, left, down, up)
directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def is_valid(pos, battlefield):
    # Check if the position is reachable
    x, y = pos
    return 0 <= x < len(battlefield) and 0 <= y < len(battlefield[0]) and battlefield[x][y] != 4

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
    
    return None

def main():
    # Load the battlefield map (64x64 grid)
    battlefield = np.full((64, 64), -1)
    
    # Randomly generate elevated terrain
    np.random.seed(0)  # For reproducibility
    elevated_positions = np.random.choice(64*64, size=512, replace=False)
    for pos in elevated_positions:
        battlefield[pos // 64][pos % 64] = 4
    
    # Define start and goal positions
    start = (0, 0)
    goal = (63, 63)
    battlefield[start[0]][start[1]] = 0
    battlefield[goal[0]][goal[1]] = 6
    
    # Run the A* algorithm
    path = astar(start, goal, battlefield)
    if path:
        print("Path found:", path)
    else:
        print("No valid path found.")

if __name__ == "__main__":
    main()
