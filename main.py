#!/usr/bin/env python3
#
#   discrete_path_demo.py
#
import numpy as np
import matplotlib
import matplotlib.pyplot as plt


# Define the possible status levels for each state.
WALL      = 0
UNKNOWN   = 1
ONDECK    = 2
PROCESSED = 3
ONPATH    = 4
START     = 5
GOAL      = 6

PATH_STATES = {
    0: 7,
    1: 8,
    2: 9,
    3: 10,
    4: 11
}

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

######################################################################
#
#   Main Code
#

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


num_robots = 2

# Define the grid with unknown states.
M = 11
N = 17

state = np.ones((M,N)) * UNKNOWN

# Populate the states.
state[ 0,0:] = WALL
state[-1,0:] = WALL
state[0:, 0] = WALL
state[0:,-1] = WALL

state[3, 4:10] = WALL
state[4,   10] = WALL
state[5,   11] = WALL
state[6,   12] = WALL
state[7,   13] = WALL
state[7:M,  7] = WALL

def get_neighbors(square):
    neighbors = list()
    i, j = square
    if i < M-1:
        neighbors.append((i+1, j))
    if i > 0:
        neighbors.append((i-1, j))
    if j < N-1:
        neighbors.append((i, j+1))
    if j > 0:
        neighbors.append((i, j-1))
    return neighbors

def manhattan_distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

from heapq import heappush, heappop

def a_star(state, start, goal, forbidden, a_factor=1):

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
    heappush(q, (a_factor * manhattan_distance(start, goal), 
                SpaceTimeCoordinate(start[0], start[1], 0)))

    while True:
        # Ran out of squares to visit without finding the goal
        if not q:
            raise RuntimeError("No path from start to goal!")

        dist, coord = heappop(q)
        dist -= a_factor * manhattan_distance(goal, coord.get_space())
        num_processed += 1

        # We found the goal
        if coord.get_space() == goal:
            # We don't want to step onto the goal if another path will pass through
            # it at some point in the future. This just looks ahead 20 time steps -
            # we'll need to change it at some point.
            for i in range(20):
                if SpaceTimeCoordinate(coord.x, coord.y, coord.time + i) in forbidden:
                    continue

            # Backtrace the optimal path from the goal to the start
            path = []
            path_length = 0
            curr = coord
            occupied = set()

            for i in range(20):
                occupied.add(SpaceTimeCoordinate(curr.x, curr.y, curr.time + i))

            while curr in parents:
                path.append(curr.get_space())
                curr = parents[curr]
                path_length += 1
                occupied.add(curr)

            path.append(curr.get_space())
            path.reverse()

            print(f"Path length: {path_length}")
            print(f"Number of states processed: {num_processed}")

            return path, occupied

        # Add all unseen neighbors to the processing queue
        visited.add(coord.get_space())
        for nbor in get_neighbors(coord.get_space()):
            new_coord = SpaceTimeCoordinate(nbor[0], nbor[1], coord.time + 1)
            if nbor not in visited and new_coord not in forbidden \
                and state[nbor] != WALL and state[nbor] != START:
                man_dist = manhattan_distance(goal, nbor)
                heappush(q, (1 + dist + new_coord.time + a_factor * man_dist, new_coord))
                parents[new_coord] = coord

        # Add the option of remaining in the same location
        new_coord = SpaceTimeCoordinate(coord.x, coord.y, coord.time + 1)
        man_dist = manhattan_distance(goal, new_coord.get_space())
        heappush(q, (1 + dist + new_coord.time + a_factor * man_dist, new_coord))
        parents[new_coord] = coord

    # Never reached
    raise RuntimeError("Error: should never be reached")

def dijkstra(state, start, goal):
    return a_star(state, start, goal, a_factor=0)

# Update/show the grid and show the S/G states labelled.
robots_start = ((9, 8),  (5, 3), (1, 15), (5, 1), (1, 1))
robots_goal  = ((5, 12), (5, 13), (9, 1), (3, 15), (6, 11))

PATH_COLORS = ['purple', 'orange', 'yellow', 'magenta', 'maroon']

for start, end in zip(robots_start, robots_goal):
    state[start] = START
    state[end]   = GOAL

forbidden = set()

paths = []

for i in range(len(robots_start)):
    path, occupied = a_star(state, robots_start[i], robots_goal[i], forbidden)
    paths.append(path)

    for coord in occupied:
        forbidden.add(coord)

for i in range(max([len(path) for path in paths])):
    for j, path in enumerate(paths):
        if i >= len(path):
            continue

        state[path[i]] = PATH_STATES[j]

        if i > 0 and path[i - 1] != path[i] and state[path[i - 1]] == PATH_STATES[j]:
            if path[i - 1] in robots_goal:
                state[path[i - 1]] = GOAL
            else:
                state[path[i - 1]] = UNKNOWN

    showgrid(state)
    input('Hit return to continue')