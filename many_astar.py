#!/usr/bin/env python3
#
#   many_astar.py
#
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import sys

from heapq import heappush, heappop

from utils import *

mappath = sys.argv[1] if len(sys.argv) > 1 else 'maps/hw1_many_robots.txt'
state, robots_start, robots_goal = load_map(mappath)

def a_star_multi(state, starts, goals, a_factor=1, visualize=False):

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
        if visualize:
            state_show = state.copy()
            for i, robot_loc in enumerate(coords):
                state_show[robot_loc] = PATH_STATES[i]
            showgrid(state_show)

        assert coords not in visited
        visited.add(coords)
        for nbor in get_neighbors_multi(coords, state):
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

        if num_processed % 100 == 0:
            print(f"Processed {num_processed} states")

    # Never reached
    raise RuntimeError("Error: should never be reached")

for i in range(len(robots_start)):
    state[robots_start[i]] = PATH_STATES[i]
    state[robots_goal[i]]  = GOAL

path = a_star_multi(state, robots_start, robots_goal, a_factor=2)
for step in path:
    for i, robot_loc in enumerate(step):
        state[robot_loc] = PATH_STATES[i]
    showgrid(state)
    for i, robot_loc in enumerate(step):
        state[robot_loc] *= -1
    input('Hit return to continue')
