#!/usr/bin/env python3
#
#   main.py
#
import numpy as np
import matplotlib.pyplot as plt

import argparse

from maze import make_maze_map, make_obstacle_map
from utils import *

from repeated_a_star import repeated_a_star

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Solve pathfinding with multiple robots')
    parser.add_argument('--map', dest='map', default='obstacle', type=str,
                        help='either "obstacle", "maze", or a map filename')
    parser.add_argument('--mapsize', dest='mapsize', default=15, type=int,
                        help='length of map for "maze" or "obstacle" map types')
    parser.add_argument('--nrobots', dest='nrobots', default=5, type=int,
                        help='number of robots for "maze" or "obstacle" map types')
    parser.add_argument('--sparsity', dest='sparsity', default=0.7, type=float,
                        help='how many obstacles are in the map, for the "obstacle" map type')
    args = parser.parse_args()

    if args.map == 'obstacle':
        state, robots_start, robots_goal = make_obstacle_map(args.mapsize, args.nrobots, density=args.sparsity)
    elif args.map == 'maze':
        state, robots_start, robots_goal = make_maze_map(args.mapsize, args.nrobots)
    else:
        state, robots_start, robots_goal = load_map(args.map)

    paths = repeated_a_star(state, robots_start, robots_goal, a_factor=1)

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