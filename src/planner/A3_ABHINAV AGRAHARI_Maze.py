import queue
from enum import Enum
import itertools
# outputing map
from matplotlib import pyplot as plt
from matplotlib import colors
import numpy as np

# 2D list in Python. Note that the first row in the 2D list is the y = 0 row (i.e. bottom-most row in the maze figure). 
# '1' indicates that the node is blocked, '0' indicates that it is free.
maze = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0]]
MIN_X = 0
MAX_X = 24
MIN_Y = 0
MAX_Y = 24
STARTING_NODE = [2,11] # define x,y position of starting position
GOAL_NODE = [23,19] # define x,y position of Exit

# Define all known actions, at an arbitrary state.
class Actions(Enum):
        MOVE_UP = 1
        MOVE_DOWN = 2
        MOVE_LEFT = 3
        MOVE_RIGHT = 4

# Closed_list:
# Store visited nodes (closed queue, in a matrix form)
visited_nodes = [[0 for i in range(25)] for j in range(25)] # a `1` inidicates position was already visited
counter = itertools.count() # counter for secondary priority in priority queue

class Node:
        '''Represents a node/cell in the search space.
           Holds (x,y) location of node, and node's parent node.
           Also holds value of evaluation function f(n) = g(n) + h(n)
        '''
        
        def __init__(self, x, y, parent) -> None:
                self.x: int = x
                self.y: int = y
                self.parent: Node = parent
                self.h_n = calculate_manhattan_distance([x,y], GOAL_NODE) # heuristic to goal 1
                # final heuristic is lesser of difference of E1 or E2 (closest goal)
                self.g_n = 0 # starting node has no initial cost
                if self.parent is not None:
                        self.g_n = self.parent.g_n + 1 # cost from starting position to this node is one more than parent
                self.f_n = self.g_n + self.h_n
        
        def position(self) -> list:
                return [self.x, self.y]



# Helper Functions
def is_in_maze(coord: list) -> bool:
        '''Checks whether a given position is within bounds of the maze'''
        x,y = coord
        x_valid = x >= MIN_X and x <=MAX_X
        y_valid = y >= MIN_Y and y <=MAX_Y
        return x_valid and y_valid

def is_empty(coord: list) -> bool:
        '''Checks whether a given position is not blocked'''
        x,y = coord
        # 2D list is accessed [row][col] ==> [y][x]
        return maze[y][x] == 0 # returns True if position is empty

def is_valid_move(new_state: list) -> bool:
        '''Checks whether moving into a new_state will be a valid move, ignoring whether node was visited'''
        # is it in the maze, empty
        # returns true if it is valid for agent to move into new_state
        return is_in_maze(new_state) and is_empty(new_state)

def is_visited(coord: list) -> bool:
        '''Checks whether a given position has already been visited'''
        x,y = coord
        # 2D list is accessed [row][col] ==> [y][x]
        return visited_nodes[y][x] == 1 # returns True if coord has not yet been visited

def is_states_equal(state_1: list, state_2: list) -> bool:
        '''Checks whether two given states are equal'''
        return state_1 == state_2

def calculate_manhattan_distance(state_1: list, state_2: list) -> int:
        '''Calculates Manhattan distance between two cells'''
        return abs(state_1[0] - state_2[0]) + abs(state_1[1] - state_2[1])

        
def get_new_state(state: list, action: Actions) -> bool:
        '''Moves agent according to action. Does not check validity of moving into new_state'''
        new_state = state.copy()
        if action == Actions.MOVE_UP:
                new_state[1] += 1
        elif action == Actions.MOVE_DOWN:
                new_state[1] -= 1
        elif action == Actions.MOVE_LEFT:
                new_state[0] -= 1
        elif action == Actions.MOVE_RIGHT:
                new_state[0] += 1
        return new_state

def get_adjacent_nodes(curr_node: Node) -> list[Node]:
        '''Gets adjacent valid nodes, ignoring visited check'''
        adjacent_nodes = []
        # Determine possible actions at given state
        for action in Actions: #FIFO, can iterate in normal order
                new_potential_state = get_new_state(curr_node.position(), action)
                adjacent_node = Node(*new_potential_state, curr_node) # Link parent
                if is_valid_move(new_potential_state):
                        adjacent_nodes.append(adjacent_node)        
        return adjacent_nodes

path = []
# ----- BACKTRACK PATH -------
def backtrack_path(current_node: Node, nodes_explored: int):
        print(f"Search Algorithim: A*")
        global path
        if current_node.position()  == GOAL_NODE:
                print(f"Agent reached goal E1 {GOAL_NODE} after exploring {nodes_explored} nodes")
                # found goal, now backtrack
                path_cost = 0
                while True:
                        path.append(current_node.position())
                        path_cost += 1
                        parent = current_node.parent
                        if parent is None:
                                break
                        current_node = parent
                path.reverse()
                print(f"Path cost: {path_cost}")
                print(f"Final path: {path}")

        else:
                print(f"Agent was not able to find the exit after {nodes_explored} moves")

# ----- A* SEARCH ------
def a_star():
        openQ = queue.PriorityQueue()
        start_node = Node(STARTING_NODE[0], STARTING_NODE[1], None)
        openQ.put((start_node.f_n, next(counter), start_node)) # Add starting node to open queue
        # A tuple of (f(n), count, node) is inserted into the priority queue.
        # The first element of the tuple allows priority queue to automatically sort.
        # The second element allows for tie-breaking when multiple nodes with the same priority exist
        # a 'counter' is used for tie-breaking above, such that it acts as a FIFO queue.
        nodes_explored = 0 # keep count of number of nodes expanded (explored / visited)
        while openQ: # while queue has items
                # pop next node to expand, check if goal state, otherwise generate adjacent valid nodes
                current_node: Node = openQ.get()[2]
                if is_visited(current_node.position()):
                        continue # node is already in closed_list, so just pop it.  ("ignore repeated states")
                # else, node has not been visited yet. Should now goal test                
                nodes_explored += 1 # Increment number of nodes explored, assuming number of nodes explored == number of nodes goal tested
                # Goal Test
                if current_node.position()  == GOAL_NODE:
                        break # Reached goal
                visited_nodes[current_node.position()[1]][current_node.position()[0]] = 1 # Put current node in closed_list
                # expand and generate adjacent nodes
                neighbours = get_adjacent_nodes(current_node)
                for next_node in neighbours:
                        if not is_visited(next_node.position()): # check that next_node isn't already in closed queue
                                openQ.put((next_node.f_n, next(counter), next_node))
        backtrack_path(current_node, nodes_explored)

a_star()

# ---- OUTPUT MAP ----
# combine all knowledge into single maze matrix, and color appropriately
# blocked, empty, starting node, goal node, pathtaken
# 1         0        2             3             4
final_representation = [row[:] for row in maze] # copy maze
final_representation[STARTING_NODE[1]][STARTING_NODE[0]] = 2
final_representation[GOAL_NODE[1]][GOAL_NODE[0]] = 3
for pos in path:
        if is_states_equal(pos, STARTING_NODE) or is_states_equal(pos, GOAL_NODE):
                continue
        final_representation[pos[1]][pos[0]] = 4

cmap = colors.ListedColormap(['w','k', 'b', 'g', 'r'])
plt.figure(figsize=(15,15))
plt.pcolor(final_representation[::],cmap=cmap,edgecolors='k', linewidths=1)
ax = plt.gca()
ax.set_xticklabels('')
ax.set_xticks(np.arange(0.5,25.5,step=1),      minor=True)
ax.set_xticklabels([str(x) for x in range(25)], minor=True)
ax.set_yticklabels('')
ax.set_yticks(np.arange(0.5,25.5,step=1),      minor=True)
ax.set_yticklabels([str(x) for x in range(25)], minor=True)
plt.xlabel('x')
plt.ylabel('y')
plt.title(f"A* Search Algorithim")
plt.show()
