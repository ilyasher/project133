from mazelib import Maze
from mazelib.generate.DungeonRooms import DungeonRooms
import numpy as np

def generate_robot_locs(array, num_robots):
    locs = list()
    while len(locs) < 2*num_robots:
        i, j = tuple(np.random.randint(low=1, high=np.min(array.shape)-1, size=2))
        if array[i, j] == 1 and (i, j) not in locs:
            locs.append((i, j))

    robots_start = tuple(locs[:num_robots])
    robots_end = tuple(locs[num_robots:])
    return robots_start, robots_end

def make_maze_map(N, num_robots):
    n = N // 2 + 1
    m1 = Maze()
    m1.generator = DungeonRooms(n, n)
    m1.generate()
    # m2 = Maze()
    # m2.generator = DungeonRooms(n, n)
    # m2.generate()

    # array = (m1.grid + m2.grid < 2).astype(int)
    array = 1 - m1.grid

    robots_start, robots_end = generate_robot_locs(array, num_robots)
    return array, robots_start, robots_end

def make_obstacle_map(N, num_robots, sparsity=0.7):
    array = np.zeros(shape=(N, N))
    array[1:-1, 1:-1] = 1

    while np.mean(array[1:-1, 1:-1]) > sparsity:
        i, j = tuple(np.random.randint(low=1, high=N-1, size=2))
        if np.random.rand() < 0.5:
            array[i-1:i+1, j-1:j+1] = 0
        else:
            array[i, j] = 0

    robots_start, robots_end = generate_robot_locs(array, num_robots)
    return array, robots_start, robots_end

