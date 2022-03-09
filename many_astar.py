#!/usr/bin/env python3
#
#   many_astar.py
#
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import sys
import time

from heapq import heappush, heappop
from queue import Queue
from functools import lru_cache

from sklearn import neighbors

from utils import *
from maze import make_maze_map, make_obstacle_map

# mappath = sys.argv[1] if len(sys.argv) > 1 else 'maps/hw1_many_robots.txt'
# state, robots_start, robots_goal = load_map(mappath)

state, robots_start, robots_goal = make_obstacle_map(20, 7, density=0.5)

def distance_array(walls, start):
    sol = walls.copy().astype(float)
    sol[:] = np.inf
    q = Queue()
    q.put((0, start))
    visited = set()
    while not q.empty():
        dist, (i, j) = q.get()
        if (i, j) in visited:
            continue
        visited.add((i, j))
        if not (0 <= i < walls.shape[0]):
            continue
        if not (0 <= j < walls.shape[1]):
            continue
        if walls[i, j]:
            continue
        sol[i, j] = dist
        for (di, dj) in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            if (i+di, j+dj) not in visited:
                q.put((dist+1, (i+di, j+dj)))
    # print(sol)
    return sol

def distance_multi(goals, dests, distances):
    s = 0
    for i in range(len(goals)):
        s += distances[goals[i]][dests[i]]
    return s

def a_star_multi(state, starts, goals, a_factor=1, visualize=False, max_steps=np.inf):

    # Trace the path to the start square from any visited square
    parents = dict()

    # Set of visited coordinates in space
    visited = set()

    unique_coords_in_q = set()

    # Number of squares we processed
    num_processed = 0

    walls = (state == WALL)
    distances = {goal : distance_array(walls, goal) for goal in goals}
    print("Constructed distance arrays")

    # Check whether solution is possible
    print(distance_multi(goals, starts, distances))
    if distance_multi(goals, starts, distances) == np.inf:
        print("Impossible map!")
        return None, 0

    # Priority queue used to process squares
    q = []

    def enqueue(new_coords, prev_dist=0, parent=None):
        dist = distance_multi(goals, new_coords, distances)
        heappush(q, (len(starts) + prev_dist + a_factor * dist, new_coords))
        if parent is not None:
            parents[new_coords] = parent
        unique_coords_in_q.add(new_coords)
        assert len(q) == len(unique_coords_in_q)

    def dequeue():
        dist, coord = heappop(q)
        dist -= a_factor * distance_multi(goals, coord, distances)
        unique_coords_in_q.remove(coord)
        assert len(q) == len(unique_coords_in_q)
        return dist, coord

    enqueue(starts)

    while True:
        # Ran out of squares to visit without finding the goal
        if not q:
            raise RuntimeError("No path from start to goal!")

        prev_dist, coords = dequeue()

        if coords in visited:
            continue

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

            return path, num_processed

        # Add all unseen neighbors to the processing queue
        if visualize:
            state_show = state.copy()
            for i, robot_loc in enumerate(coords):
                state_show[robot_loc] = PATH_STATES[i]
            showgrid(state_show)

        assert coords not in visited
        visited.add(coords)
        neighbors = get_neighbors_multi(coords, state)
        for nbor in neighbors:
            if nbor in visited:
                continue
            # I think there might be a bug with this
            if nbor in unique_coords_in_q:
                continue
            enqueue(nbor, prev_dist=prev_dist, parent=coords)

        if num_processed % 500 == 0:
            print(f"Processed {num_processed} states")

        if num_processed >= max_steps:
            return None, num_processed

    # Never reached
    raise RuntimeError("Error: should never be reached")

for i in range(len(robots_start)):
    state[robots_start[i]] = PATH_STATES[i]
    state[robots_goal[i]]  = GOAL

showgrid(state)
start_time = time.time()
path, n_steps = a_star_multi(state, robots_start, robots_goal, a_factor=10, visualize=False)
end_time = time.time()
print(f"Found solution in {end_time - start_time} seconds")
# print(f"Solution is {len(path)} steps long.")
input("Hit return to continue.")

total_time = 5 # seconds
max_time_per_step = 2
time_per_step = min(max_time_per_step, total_time / len(path))

for step in path:
    for i, robot_loc in enumerate(step):
        state[robot_loc] = PATH_STATES[i]
    showgrid(state)
    plt.pause(time_per_step)
    for i, robot_loc in enumerate(step):
        state[robot_loc] *= -1
input("Hit return to exit")
