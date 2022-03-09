#!/usr/bin/env python3
#
#   many_astar.py
#
import numpy as np

from heapq import heappush, heappop
from queue import Queue

from utils import *

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

def joint_a_star(state, starts, goals, a_factor=10, visualize=False, max_steps=np.inf):

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
    if distance_multi(goals, starts, distances) == np.inf:
        print("Impossible map!")
        return None

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
            paths = [[] for robot in goals]
            path_length = 0
            curr = coords

            while curr in parents:
                for i in range(len(curr)):
                    paths[i].append(curr[i])
                curr = parents[curr]
                path_length += 1

            for i in range(len(curr)):
                paths[i].append(curr[i])
            for path in paths:
                path.reverse()

            print(f"Path length: {path_length}")
            print(f"Number of states processed: {num_processed}")

            return paths

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
            return None

    # Never reached
    raise RuntimeError("Error: should never be reached")
