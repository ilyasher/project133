# Path finder for multiple robots

```
usage: main.py [-h] [--map MAP] [--mapsize MAPSIZE] [--nrobots NROBOTS] [--sparsity SPARSITY] [--algo {repeated,joint}]

Solve pathfinding with multiple robots

optional arguments:
  -h, --help            show this help message and exit
  --map MAP             either "obstacle", "maze", or a map filename
  --mapsize MAPSIZE     length of map for "maze" or "obstacle" map types
  --nrobots NROBOTS     number of robots for "maze" or "obstacle" map types
  --sparsity SPARSITY   how many obstacles are in the map, for the "obstacle" map type
  --algo {repeated,joint}
                        which algorithm to use - either "repeated" or "joint"
```
Example usage:
```
main.py --map maps/bendy_swap.txt
main.py --map maps/puzzle2.txt --algo joint
main.py --map maze --mapsize 20 --nrobots 5 --algo joint
main.py --map obstacle --mapsize 15 --sparsity 0.8
```