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

def a_star(state, start, goal, num_robots, a_factor=1):

    # Copy of input map which we will manipulate
    sol = state.copy()

    # Trace the path to the start square from any visited square
    parents = dict()

    # Number of squares we processed
    num_processed = 0

    # Priority queue used to process squares
    q = []
    heappush(q, (a_factor*manhattan_distance(start, goal), start))

    while True:

        # Ran out of squares to visit without finding the goal
        if not q:
            raise RuntimeError("No path from start to goal!")

        dist, square = heappop(q)
        #for i in range(num_robots):
        dist -= a_factor*manhattan_distance(goal, square)
        num_processed += 1

        # We found the goal
        if square == goal:
            # Backtrace the optimal path from the goal to the start
            path_length = 0
            curr = square
            while curr in parents:
                sol[curr] = ONPATH
                curr = parents[curr]
                path_length += 1
            sol[curr] = ONPATH
            print(f"Path length: {path_length}")
            print(f"Number of states processed: {num_processed}")
            break

        # Add all unseen neighbors to the processing queue
        sol[square] = PROCESSED
        for nbor in get_neighbors(square):
            if sol[nbor] == UNKNOWN:
                man_dist = manhattan_distance(goal, nbor)
                heappush(q, (1 + dist + a_factor*man_dist, nbor))
                sol[nbor] = ONDECK
                parents[nbor] = square

    sol[sol == PROCESSED] = UNKNOWN
    sol[sol == ONDECK]    = UNKNOWN
    return sol

def dijkstra(state, start, goal):
    return a_star(state, start, goal, a_factor=0)

# Update/show the grid and show the S/G states labelled.
robots_start = ((5, 4),  (5, 3))
robots_goal  = ((5, 12), (5, 13))
#start = (5, 4)
#goal = (5, 12)

# METHOD 1: DO A* separately so that the two paths don't intersect
sol = a_star(state, robots_start[0], robots_goal[0], 1)
sol = a_star(sol,   robots_start[0], robots_goal[1], 1)

for start, end in zip(robots_start, robots_goal):
    sol[start] = START
    sol[end]   = GOAL

showgrid(sol)
input('Hit return to continue')
