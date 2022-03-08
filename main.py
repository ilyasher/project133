#!/usr/bin/env python3
#
#   main.py
#
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import sys

from heapq import heappush, heappop

from utils import *

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

mappath = sys.argv[1] if len(sys.argv) > 1 else 'maps/hw1_many_robots.txt'
state, robots_start, robots_goal = load_map(mappath)

def a_star(state, start, goal, forbidden, a_factor=1):

    # Trace the path to the start square from any visited square
    parents = dict()

    # Set of visited coordinates in space
    visited = set()

    unique_coords_in_q = set()

    # Number of squares we processed
    num_processed = 0

    q = []

    def enqueue(new_coord, prev_dist=0, parent=None):
        man_dist = manhattan_distance(goal, new_coord.get_space())
        heappush(q, (1 + prev_dist + new_coord.time + a_factor * man_dist, new_coord))
        if parent is not None:
            parents[new_coord] = parent
        unique_coords_in_q.add(new_coord.get_space())
        assert len(q) == len(unique_coords_in_q)

    def dequeue():
        dist, coord = heappop(q)
        dist -= a_factor * manhattan_distance(goal, coord.get_space())
        unique_coords_in_q.remove(coord.get_space())
        assert len(q) == len(unique_coords_in_q)
        return dist, coord

    # Priority queue used to process squares
    enqueue(SpaceTimeCoordinate(start[0], start[1], 0))

    while True:
        # Ran out of squares to visit without finding the goal
        if not q:
            raise RuntimeError("No path from start to goal!")

        dist, coord = dequeue()
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
        visited.add(coord)
        for nbor in get_neighbors(coord.get_space(), state):
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

            enqueue(new_coord, prev_dist=dist, parent=coord)

        # Add the option of remaining in the same location
        new_coord = SpaceTimeCoordinate(coord.x, coord.y, coord.time + 1)
        if new_coord not in forbidden:
            if new_coord.get_space() not in unique_coords_in_q:
                enqueue(new_coord, prev_dist=dist, parent=coord)

    # Never reached
    raise RuntimeError("Error: should never be reached")

for i in range(len(robots_start)):
    state[robots_start[i]] = PATH_STATES[i]
    state[robots_goal[i]]  = GOAL

forbidden = set()

paths = []

for i in range(len(robots_start)):
    path, occupied = a_star(state, robots_start[i], robots_goal[i], forbidden)
    paths.append(path)

    for coord in occupied:
        forbidden.add(coord)

showgrid(state)
input("Solution found. Hit return to continue.")
for i in range(max([len(path) for path in paths])):
    for j, path in enumerate(paths):
        state[path[min(i, len(path)-1)]] = PATH_STATES[j]
    showgrid(state)
    plt.pause(0.1)
    for j, path in enumerate(paths):
        state[np.isin(state, PATH_STATES)] *= -1
input("Hit return to exit")