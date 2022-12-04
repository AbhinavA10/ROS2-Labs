import numpy as np
import matplotlib.pyplot as plt
import yaml
from scipy.ndimage import rotate
import heapq #Priority queue implementation
from collections import deque as queue
# import bezier

# List containing directions for neighboring indices (i,j) = (x,y)
dirVal = [ (0,1), (1,1), (1,0),(0,-1), (1,-1), (-1,0), (-1, -1), (-1, 1)]

def is_legal(maze, visited, nxt_pos):
    """Function to check if a cell to is be visited or not"""

    #Check if index lies is out of bounds
    if (nxt_pos[0] < 0 or nxt_pos[1] < 0 or nxt_pos[0] >= maze.shape[0] or nxt_pos[1] >= maze.shape[1]):
        return False

    #Check if index is visited
    if (visited[nxt_pos[0],nxt_pos[1]]):
        return False

    #Don't move to nodes with 85% chance of being an obstacle
    if (maze[nxt_pos[0], nxt_pos[1]] > 85.0): 
        return False
    return True

class Node():
    """A node class for A* Pathfinding"""
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0  #Cost

    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.position < other.position
        
def heuristic(x1,y1,x2,y2): 
    """Heuristic function based on the manhattan distance between the points x1,y1 x2,y2"""
    return abs(x1-x2) + abs(y1-y2) 

def heuristic_diag(x1,y1,x2,y2): 
    """heuristic function based on diagonal (octile) distance"""
    dx = abs(x1 - x2)
    dy = abs(y1 - y2)
    D = 1
    D2 = 1.4
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy) #http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#diagonal-distance

def A_star(maze: np.ndarray, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    end_node = Node(None, end)
    # Check start and end are within the map
    if (start[0] >= 0 and start[1] >= 0 and start[0] < maze.shape[0] and start[1] < maze.shape[1]):
        if (end[0] >= 0 and end[1] >= 0 and end[0] < maze.shape[0] and end[1] < maze.shape[1]):
            # Initialize both open and closed list
            open_list = []
            path = queue()
            visited = np.full(maze.shape, False)
            visited[start_node.position[0],start_node.position[1]] = True
            # Add the start node
            heapq.heappush(open_list, (0 ,(start_node, path))) # Priority queue using heappush, adding the starting node 

            # Loop until you find the end node
            while len(open_list) > 0:
                # Get the current node
                item = heapq.heappop(open_list) 
                current_node = item[1][0]
                cost = current_node.g
                p = item[1][1].copy()
                p.append(current_node.position)
                # Found the goal
                if current_node == end_node:
                    # Return the shortest path found along with heuristic
                    return p, heuristic_diag(start_node.position[0], start_node.position[1], end_node.position[0], end_node.position[0])
                    # Smooth A* path using bezier curve
                    # curve = bezier.Curve.from_nodes(np.asfortranarray(path.T))
                    # s_vals = np.linspace(0.0, 1.0, 100)
                    # test = np.array(curve.evaluate_multi(s_vals))
                    # return test.T, cost

                # Generate children, which are the neighboring nodes. Should use 4 or 8 points connectivity for a grid.
                for adj in dirVal:
                    move_cost = 1
                    if abs(adj[0]) - abs(adj[1]) == 0:
                        move_cost = 1.4
                    nxt_pos = tuple(np.add(current_node.position ,adj))

                    if (is_legal(maze, visited, nxt_pos)):
                        prob = maze[nxt_pos[0],nxt_pos[1]] #Probability of a point on the grid
                        move_cost += prob/80 #scaling found arbitrarily through testing
                        est_cost = cost + move_cost + (0.25 + prob/100) * heuristic_diag(nxt_pos[0], nxt_pos[1], end_node.position[0], end_node.position[0]) #estimated cost based on heuristic
                        nxt_node = Node(None, nxt_pos)
                        nxt_node.g = cost + move_cost
                        heapq.heappush(open_list, (est_cost, (nxt_node, p))) #Push all generated points to the priority queue
                        visited[nxt_pos[0],nxt_pos[1]] = True
    # No path found
    return [], 0
        

def find_path(start: tuple, goal: tuple, occupancy_grid):
    
    # Compute the path with A*
    path, cost = A_star(occupancy_grid, start, goal)
    path = np.array(list(path))

    return path, cost
