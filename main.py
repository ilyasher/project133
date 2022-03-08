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
    color = np.ones((M,N,4))
    is_path = state < 0
    state = np.abs(state)
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
            c = matplotlib.colors.to_rgba_array(c)[0]
            if is_path[m, n]:
                c[-1] = 0.5
            color[m,n] = c

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

    def __repr__(self):
        return f"SpaceTimeCoordinate: x:{self.x}, y:{self.y}, time:{self.time}"

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

    return state, robots_start, robots_goal

mappath = sys.argv[1] if len(sys.argv) > 1 else 'maps/map1.txt'
state, robots_start, robots_goal = load_map(mappath)
M, N = state.shape

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

    unique_coords_in_q = set()

    # Number of squares we processed
    num_processed = 0

    # Priority queue used to process squares
    q = []
    heappush(q, (a_factor * manhattan_distance(start, goal),
                SpaceTimeCoordinate(start[0], start[1], 0)))
    print(start)
    unique_coords_in_q.add(start)

    while True:
        # Ran out of squares to visit without finding the goal
        if not q:
            raise RuntimeError("No path from start to goal!")

        dist, coord = heappop(q)
        dist -= a_factor * manhattan_distance(goal, coord.get_space())
        num_processed += 1
        unique_coords_in_q.remove(coord.get_space())

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
        # visited.add(coord.get_space())
        visited.add(coord)
        for nbor in get_neighbors(coord.get_space()):
            if nbor in unique_coords_in_q:
                continue
            new_coord = SpaceTimeCoordinate(nbor[0], nbor[1], coord.time + 1)
            if new_coord in visited:
                continue
            if new_coord in forbidden:
                continue
            if any(map(lambda x: x[1] == new_coord, q)):
                continue
            if state[nbor] == WALL:
                continue

            assert nbor not in visited

            # Don't pass thru each other
            # This is technically not correct, it is slightly too harsh
            new_coord2 = SpaceTimeCoordinate(nbor[0], nbor[1], coord.time)
            new_coord3 = SpaceTimeCoordinate(coord.x, coord.y, coord.time + 1)
            if new_coord2 in forbidden and new_coord3 in forbidden:
                continue

            man_dist = manhattan_distance(goal, nbor)
            heappush(q, (1 + dist + new_coord.time + a_factor * man_dist, new_coord))
            parents[new_coord] = coord
            unique_coords_in_q.add(new_coord.get_space())

        # Add the option of remaining in the same location
        new_coord = SpaceTimeCoordinate(coord.x, coord.y, coord.time + 1)
        if new_coord not in forbidden:
            man_dist = manhattan_distance(goal, new_coord.get_space())
            heappush(q, (1 + dist + new_coord.time + a_factor * man_dist, new_coord))
            parents[new_coord] = coord
            unique_coords_in_q.add(new_coord.get_space())

    # Never reached
    raise RuntimeError("Error: should never be reached")

def dijkstra(state, start, goal):
    return a_star(state, start, goal, a_factor=0)

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
        state[path[min(i, len(path)-1)]] = PATH_STATES[j]
    showgrid(state)
    plt.pause(0.1)
    for j, path in enumerate(paths):
        # state[PATH_STATES[0] <= state <= PATH_STATES[-1]] *= -1
        state[np.isin(state, PATH_STATES)] *= -1
        # state[PATH_STATES[0] <= state <= PATH_STATES[-1]] *= -1
        # state[path[min(i, len(path)-1)]] = PATH_STATES[j]

        # if i > 0 and path[i - 1] != path[i] and state[path[i - 1]] == PATH_STATES[j]:
        #     if path[i - 1] in robots_goal:
        #         state[path[i - 1]] = GOAL
        #     else:
        #         state[path[i - 1]] = UNKNOWN
input("Hit return to exit")