#!/usr/bin/env python3
#
#   discrete_path_demo.py
#
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import sys


# Define the possible status levels for each state.
WALL      = 0
UNKNOWN   = 1
ONDECK    = 2
PROCESSED = 3
ONPATH    = 4
START     = 5
GOAL      = 6

PATH_STATES = [7, 8, 9, 10, 11]

######################################################################
#
#   showgrid(M,N)
#
#   Create a figure for an M (rows) x N (column) grid.  The X-axis
#   will be the columns (to the right) and the Y-axis will be the rows
#   (top downward).
#
def showgrid(state):

    # Grab the dimensions.
    M = np.size(state, axis=0)
    N = np.size(state, axis=1)

    # Close the old figure.
    plt.close()

    # Create the figure and axes.
    fig = plt.figure()
    ax  = plt.axes()

    # turn off the axis labels
    ax.axis('off')

    # Draw the grid, zorder 1 means draw after zorder 0 elements.
    for m in range(M+1):
        ax.axhline(m, lw=1, color='b', zorder=1)
    for n in range(N+1):
        ax.axvline(n, lw=1, color='b', zorder=1)

    # Create the color range.  There are clearly more elegant ways...
    color = np.ones((M,N,3))
    for m in range(M):
        for n in range(N):
            state_to_color = {
                WALL: 'black',
                UNKNOWN: 'white',
                ONDECK: 'white',
                PROCESSED: 'lightblue',
                ONPATH: 'purple',
                START: 'green',
                GOAL: 'red',
                PATH_STATES[0]: 'purple',
                PATH_STATES[1]: 'orange',
                PATH_STATES[2]: 'magenta',
                PATH_STATES[3]: 'yellow',
                PATH_STATES[4]: 'maroon'
            }
            if state[m, n] not in state_to_color:
                c = 'red'
            else:
                c = state_to_color[state[m, n]]
            color[m,n,0:3] = np.array(matplotlib.colors.to_rgb(c))

    # Draw the boxes
    ax.imshow(color, aspect='equal', interpolation='none',
              extent=[0, N, 0, M], zorder=0)

    # Force the figure to pop up.
    plt.pause(0.001)


class SpaceTimeCoordinate:
    def __init__(self, x, y, time):
        self.x = x
        self.y = y
        self.time = time

    def get_space(self):
        return (self.x, self.y)

    def __lt__(self, other):
        return (self.x, self.y, self.time) < (other.x, other.y, other.time)

    def __gt__(self, other):
        return (self.x, self.y, self.time) > (other.x, other.y, other.time)

    def __eq__(self, other):
        return (self.x, self.y, self.time) == (other.x, other.y, other.time)

    def __hash__(self):
        return hash((self.x, self.y, self.time))

def load_map(filepath):
    with open(filepath) as f:
        lines = f.readlines()
    #assert len(set(map(len, lines))) == 1
    M = len(lines)
    N = len(lines[0])

    starts = dict()
    goals = dict()

    state = np.ones((M, N)) * UNKNOWN

    for i, line in enumerate(lines):
        for j, c in enumerate(line):
            if c == '#':
                state[i, j] = WALL
            elif c.isalpha():
                if c.islower():
                    starts[c] = (i, j)
                else:
                    goals[c.lower()]  = (i, j)

    robots_start = list()
    robots_goal  = list()
    for robot in starts:
        robots_start.append(starts[robot])
        robots_goal.append(goals[robot])

    return state, tuple(robots_start), tuple(robots_goal)

mappath = sys.argv[1] if len(sys.argv) > 1 else 'maps/map1.txt'
state, robots_start, robots_goal = load_map(mappath)
M, N = state.shape

def get_neighbors(square):
    neighbors = list()
    i, j = square
    neighbors.append(square)
    if i < M-1:
        neighbors.append((i+1, j))
    if i > 0:
        neighbors.append((i-1, j))
    if j < N-1:
        neighbors.append((i, j+1))
    if j > 0:
        neighbors.append((i, j-1))
    neighbors = list(filter(lambda square: state[square] != WALL, neighbors))
    return neighbors

# Create a list of all possible combinations of squares for the next move
def get_neighbors_multi(squares):
    ret = [[]]

    squares = list(squares)
    for i, square in enumerate(squares):
        new = list()

        # Add each square's neighbor only if it doesn't overlap with an
        # existing square. (So they don't pass thru each other)
        temp = squares[i]
        squares[i] = ...
        for neighbor in get_neighbors(square):
            if neighbor in squares :
                continue

            new += [r + [neighbor] for r in ret if neighbor not in r]
        squares[i] = temp

        ret = new

    # Convert to list of tuple
    ret = list(map(tuple, ret))

    return ret

def manhattan_distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def manhattan_distance_multi(a, b):
    ret = 0
    for i in range(len(a)):
        ret += manhattan_distance(a[i], b[i])
    return ret

from heapq import heappush, heappop

def a_star_multi(state, starts, goals, a_factor=1, visualize=False):

    # Copy of input map which we will manipulate
    sol = state.copy()

    # Trace the path to the start square from any visited square
    parents = dict()

    # Set of visited coordinates in space
    visited = set()

    # Number of squares we processed
    num_processed = 0

    # Priority queue used to process squares
    q = []
    heappush(q, (a_factor * manhattan_distance_multi(starts, goals), starts))

    while True:
        # Ran out of squares to visit without finding the goal
        if not q:
            raise RuntimeError("No path from start to goal!")

        dist, coords = heappop(q)
        dist -= a_factor * manhattan_distance_multi(goals, coords)
        num_processed += 1

        # We found the goal
        if coords == goals:

            # Backtrace the optimal path from the goal to the start
            path = []
            path_length = 0
            curr = coords

            while curr in parents:
                path.append(curr)
                curr = parents[curr]
                path_length += 1

            path.append(curr)
            path.reverse()

            print(f"Path length: {path_length}")
            print(f"Number of states processed: {num_processed}")

            return path

        # Add all unseen neighbors to the processing queue
        #print(coords)
        if visualize:
            state_show = state.copy()
            for i, robot_loc in enumerate(coords):
                state_show[robot_loc] = PATH_STATES[i]
            showgrid(state_show)
        assert coords not in visited
        visited.add(coords)
        print(len(visited), len(q), len(set(q)), len(coords), len(get_neighbors_multi(coords)))
        #print(q)
        for nbor in get_neighbors_multi(coords):
            if nbor in visited:
                continue
            if any(map(lambda x: x[1] == nbor, q)):
                continue
            if any(map(lambda x: state[x] == WALL, nbor)):
                continue
            assert nbor not in visited
            man_dist = manhattan_distance_multi(goals, nbor)
            heappush(q, (len(starts) + dist + a_factor * man_dist, nbor))
            parents[nbor] = coords

        # Add the option of remaining in the same location
        #new_coord = SpaceTimeCoordinate(coord.x, coord.y, coord.time + 1)
        #man_dist = manhattan_distance(goal, new_coord.get_space())
        #heappush(q, (1 + dist + new_coord.time + a_factor * man_dist, new_coord))
        #parents[new_coord] = coord

    # Never reached
    raise RuntimeError("Error: should never be reached")

def dijkstra_multi(state, start, goal):
    return a_star_multi(state, start, goal, a_factor=0)

PATH_COLORS = ['purple', 'orange', 'yellow', 'magenta', 'maroon']

for start, end in zip(robots_start, robots_goal):
    state[start] = START
    state[end]   = GOAL

state_save = state.copy()
path = a_star_multi(state, robots_start, robots_goal, a_factor=2)
for step in path:
    state_show = state_save.copy()
    for i, robot_loc in enumerate(step):
        state_show[robot_loc] = PATH_STATES[i]
    showgrid(state_show)
    input('Hit return to continue')

# for i in range(max([len(path) for path in paths])):
#     for j, path in enumerate(paths):
#         if i >= len(path):
#             continue

#         state[path[i]] = PATH_STATES[j]

#         if i > 0 and path[i - 1] != path[i] and state[path[i - 1]] == PATH_STATES[j]:
#             if path[i - 1] in robots_goal:
#                 state[path[i - 1]] = GOAL
#             else:
#                 state[path[i - 1]] = UNKNOWN

#     showgrid(state)
#     input('Hit return to continue')