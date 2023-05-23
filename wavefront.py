from queue import PriorityQueue
import time

def heuristic(node, goal):
    x1, y1 = node
    x2, y2 = goal
    return abs(x1 - x2) + abs(y1 - y2)

def a_star_algorithm(grid, start, goal):
    start_time = time.time()
    rows = len(grid)
    cols = len(grid[0])

    # Initialize distances and parent dictionary
    distances = {}
    parent = {}
    for i in range(rows):
        for j in range(cols):
            distances[(i, j)] = float('inf')
    distances[start] = 0

    # Create priority queue
    pq = PriorityQueue()
    pq.put((0, start))

    while not pq.empty():
        current_distance, current_node = pq.get()

        if current_node == goal:
            # Build path from goal to start
            path = []
            while current_node in parent:
                path.append(current_node)
                current_node = parent[current_node]
            path.append(start)
            path.reverse()
            print("A Star time executed : " + str((time.time() - start_time)*1000))
            return path

        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            next_x = current_node[0] + dx
            next_y = current_node[1] + dy

            if 0 <= next_x < rows and 0 <= next_y < cols and grid[next_x][next_y] == 0:
                distance = distances[current_node] + 1

                if distance < distances[(next_x, next_y)]:
                    distances[(next_x, next_y)] = distance
                    priority = distance + heuristic((next_x, next_y), goal)
                    pq.put((priority, (next_x, next_y)))
                    parent[(next_x, next_y)] = current_node
    print("A Star time executed : " + str((time.time() - start_time)*1000))
    return None  # No path found

def dijkstra_algorithm(grid, start, goal):
    start_time = time.time()
    rows = len(grid)
    cols = len(grid[0])

    # Initialize distances and parent dictionary
    distances = {}
    parent = {}
    for i in range(rows):
        for j in range(cols):
            distances[(i, j)] = float('inf')
    distances[start] = 0

    # Create priority queue
    pq = PriorityQueue()
    pq.put((0, start))

    while not pq.empty():
        current_distance, current_node = pq.get()

        if current_node == goal:
            # Build path from goal to start
            path = []
            while current_node in parent:
                path.append(current_node)
                current_node = parent[current_node]
            path.append(start)
            path.reverse()
            print("Dijkstra time executed : " + str((time.time() - start_time)*1000))
            return path

        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            next_x = current_node[0] + dx
            next_y = current_node[1] + dy

            if 0 <= next_x < rows and 0 <= next_y < cols and grid[next_x][next_y] == 0:
                distance = distances[current_node] + 1

                if distance < distances[(next_x, next_y)]:
                    distances[(next_x, next_y)] = distance
                    pq.put((distance, (next_x, next_y)))
                    parent[(next_x, next_y)] = current_node
    print("Dijkstra time executed : " + str((time.time() - start_time)*1000))
    return None  # No path found

def wavefront_algorithm(grid, start, goal):
    start_time = time.time()
    # Define the wavefront queue and set the starting cell with a value of 1
    wavefront = [start]
    grid[start[0]][start[1]] = 1

    # Continue until the goal cell is reached
    while wavefront:
        current_cell = wavefront.pop(0)
        x, y = current_cell[0], current_cell[1]

        # Explore the neighboring cells
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            new_x, new_y = x + dx, y + dy

            # Check if the neighboring cell is within the grid and not an obstacle
            if 0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]) and grid[new_x][new_y] == 0:
                # Assign the value of the current cell + 1 to the neighboring cell
                grid[new_x][new_y] = grid[x][y] + 1
                wavefront.append((new_x, new_y))

                # Check if the goal cell is reached
                if (new_x, new_y) == goal:
                    # Build the path from the goal cell back to the start cell
                    path = [(new_x, new_y)]
                    while path[-1] != start:
                        x, y = path[-1]
                        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                            neighbor_x, neighbor_y = x + dx, y + dy
                            if 0 <= neighbor_x < len(grid) and 0 <= neighbor_y < len(grid[0]) and grid[neighbor_x][neighbor_y] == grid[x][y] - 1:
                                path.append((neighbor_x, neighbor_y))
                                break
                    path.reverse()
                    print("Wavefront time executed : " + str((time.time() - start_time)*1000))
                    return path
    print("Wavefront time executed : " + str((time.time() - start_time)*1000))
    return None  # No path found

grid = [
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0],
    [0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0],
    [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0],
    [0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0],
    [0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0],
    [0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]
]

start = (0, 0)
goal = (0, 1) # (row, column)
# (8,14) , (14,14) , (1,14) , (0, 14)

a_star_result = a_star_algorithm(grid, start, goal)
print("A Star : " + str(a_star_result))

dijkstra_result = dijkstra_algorithm(grid, start, goal)
print("Dijkstra : " + str(dijkstra_result)) 

wavefront_result = wavefront_algorithm(grid, start, goal)
print("Wavefront : " + str(wavefront_result))