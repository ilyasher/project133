import numpy as np
import matplotlib
import matplotlib.pyplot as plt

WALL      = 0
UNKNOWN   = 1
ONDECK    = 2
PROCESSED = 3
ONPATH    = 4
START     = 5
GOAL      = 6

PATH_STATES = [7, 8, 9, 10, 11]

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
            c = matplotlib.colors.to_rgba_array(c).flatten()
            if is_path[m, n]:
                c[-1] = 0.5
            color[m,n] = c

    # Draw the boxes
    ax.imshow(color, aspect='equal', interpolation='none',
              extent=[0, N, 0, M], zorder=0)

    # Force the figure to pop up.
    plt.pause(0.001)

def get_neighbors(square, state):
    M, N = state.shape
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
    if state is not None:
        neighbors = list(filter(lambda square: state[square] != WALL, neighbors))
    return neighbors

# Create a list of all possible combinations of squares for the next move
def get_neighbors_multi(squares, state):
    ret = [[]]

    squares = list(squares)
    for i, square in enumerate(squares):
        new = list()

        # Add each square's neighbor only if it doesn't overlap with an
        # existing square. (So they don't pass thru each other)
        temp = squares[i]
        squares[i] = ...
        for neighbor in get_neighbors(square, state=state):
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
