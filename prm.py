import math
import numpy as np
import copy
import networkx as nx
from optparse import OptionParser
import rasterio
import time
import matplotlib.pyplot as plt

STAT_OBSTACLE=1
STAT_NORMAL=0

class RoadMap():

    def __init__(self, map):
        self.map = map  # Occupancy grid 2D numpy array
        self.rows, self.cols = map.shape


    def description(self):
        print("Shape: ({} x {})".format(self.rows, self.cols))


    def isValidCell(self, row, col):
        if row < 0 or row >= self.rows or col < 0 or col >= self.cols:
            return False
        return True


    def isNotObstacle(self, row, col):
        return self.map[row, col] != STAT_OBSTACLE


    def distance(self, cell1, cell2, method="euclidean"):
        if method == "euclidean":
            return ((float(cell2[0]) - float(cell1[0]))**2 + (float(cell2[1]) - float(cell1[1]))**2)**0.5 
        else:
            print("distance measurement {} not supported!")
            return -9999.0

    
    def isValidPath(self, cell1, cell2):
        steps = max(abs(cell1[0] - cell2[0]), abs(cell1[1] - cell2[1]))
        xs = np.linspace(cell1[0],cell2[0],steps+1)
        ys = np.linspace(cell1[1],cell2[1],steps+1)
        for i in range(1, steps):
            if not self.isNotObstacle(math.ceil(xs[i]), math.ceil(ys[i])):
                return False
        return True


    def plot(self):
        graphRows, graphCols = zip(*self.G.nodes)
        f, ax = plt.subplots()
        ax.scatter(graphCols, graphRows, s=10, color="tab:green")
        if self.manualPoints is not None:
            manualRows, manualCols = zip(*self.manualPoints)
            ax.scatter(manualCols, manualRows, s=10, color="tab:purple")
        for edge in self.G.edges:
            ax.plot([edge[0][1], edge[1][1]], [edge[0][0], edge[1][0]], linewidth=0.1, color="tab:green", linestyle="--")
        return ax



class PRM(RoadMap):
    
    def __init__(self, map, nSamples=100, neighborhoodDist=100.0, manualPoints=None):
        # nSamples: number of sampling points. Default 100
        # neighborhoodDist: neighborhood distance. Default 100.0
        # manualPoints: list of 2-elem tuples of (row, col) that are included in the PRM

        RoadMap.__init__(self, map)

        self.nSamples = nSamples
        self.manualPoints = manualPoints
        self.neighborhoodDist = neighborhoodDist
        self.G = nx.Graph()


    def description(self):
        RoadMap.description(self)
        print("Number of samples: {}".format(self.nSamples))
        print("Number of manual points: {}".format(len(self.manualPoints))) if self.manualPoints is not None else print("Number of manual points: 0")
        print("Neighborhood distance: {}".format(self.neighborhoodDist))
        print("Roadmap graph:")
        print("    Number of nodes: {}".format(len(self.G.nodes)))
        print("    Number of edges: {}".format(self.G.number_of_edges()))


    def save(self, outfile):
        # Save pickled roadmap graph
        nx.write_gpickle(self.G, outfile)


    def load(self, infile):
        # Load pickled roadmap graph
        self.G = nx.read_gpickle(infile)


    def learn(self):
        # Learning stage: run once

        # Add manual points
        if self.manualPoints is not None:
            for cell in self.manualPoints:
                if self.isValidCell(cell[0], cell[1]) and self.isNotObstacle(cell[0], cell[1]):
                    self.G.add_node(cell)
        
        # Add random samples
        while len(self.G.nodes) < self.nSamples:
            cell = (np.random.randint(0, self.rows),np.random.randint(0, self.cols)) # Random points
            if self.isValidCell(cell[0], cell[1]) and self.isNotObstacle(cell[0], cell[1]): # Not an obstacle point
                self.G.add_node(cell)

        # Collision detection in the neighborhood, adding edges
        for node1 in self.G.nodes:
            for node2 in self.G.nodes:
                if node1==node2:
                    continue
                dist = self.distance(node1, node2)
                if dist < self.neighborhoodDist and self.isValidPath(node1, node2):
                    self.G.add_edge(node1, node2, weight=dist) 

    
    def findPath(self, startCell, goalCell):

        # Add start, goal to copy of roadmap
        tempG = copy.deepcopy(self.G)
        tempG.add_node(startCell)
        tempG.add_node(goalCell)
        for node1 in [startCell, goalCell]:
            for node2 in tempG.nodes:
                dist = self.distance(node1, node2)
                if dist < self.neighborhoodDist and self.isValidPath(node1, node2):
                    tempG.add_edge(node1, node2, weight=dist)

        # Solve path with networkx
        path = nx.shortest_path(tempG, source=startCell, target=goalCell)
        distance = nx.path_weight(tempG, path, weight="weight")
        return path, distance



def main():

    parser = OptionParser()

    # Required files
    parser.add_option("-r", "--raster_file",
            help="Path to input raster occupancy grid.",
            default="full.tif")
    parser.add_option("-f", "--roadmap_file",
            help="Path to pickled roadmap.")

    # Optional files
    parser.add_option("-p", "--points_file",
            help="Path to input roadmap points.")
    parser.add_option("-m", "--map_plot",
            help="Path to save plotted roadmap, solution path.")

    # Build roadmap
    parser.add_option("-b", "--build_roadmap",
            help="Should build roadmap from scratch and save. Otherwise, loads pickled graph.",
            default=False, action="store_true")
    parser.add_option("-n", "--num_samples",
            help="Number of PRM sample points.",
            default=100)
    parser.add_option("-d", "--neighborhood_distance",
            help="Distance to connect neighbor nodes.",
            default=100.0)

    # Solve path
    parser.add_option("-s", "--start_cell",
            help="Coordinates of start (row,col).")
    parser.add_option("-g", "--goal_cell",
            help="Coordinates of goal cell (row,col).")

    options, args = parser.parse_args()

    rasterFile = options.raster_file
    pickleFile = options.roadmap_file

    pointsFile = options.points_file
    mapFile = options.map_plot

    buildRoadmap = options.build_roadmap
    nSamples = int(options.num_samples)
    neighborhoodDist = float(options.neighborhood_distance)

    startCell = options.start_cell
    goalCell = options.goal_cell

    if rasterFile is None:
        print("Requires a raster occupancy grid.\nExiting...")
        exit(1)
    if pickleFile is None:
        print("Requires a pickle file to save/load roadmap.\nExiting...")
        exit(1)
    solvePath = False
    if startCell is not None or goalCell is not None:
        if startCell is None or goalCell is None:
            print("To solve a path, need BOTH start (-s) and goal (-g)\nExiting...")
            exit(1)
        solvePath = True

    if solvePath:
        startCell = tuple([int(c) for c in startCell.split(',')])
        goalCell = tuple([int(c) for c in goalCell.split(',')])

    print("")
    print("Options")
    print("-------")
    print("Raster occupancy grid: {}".format(rasterFile))
    print("Pickle to save/load roadmap: {}".format(pickleFile))
    print("[Optiona] manual points: {}".format(pointsFile))
    print("Building or solving?")
    print("    Building new roadmap.") if buildRoadmap else print("    Loading pickled roadmap.")
    print("    Using roadmap to solve path.") if solvePath else print("    Not solving a path")
    if solvePath:
        print("Start = ({}, {})   goal = ({}, {})".format(startCell[0], startCell[1], goalCell[0], goalCell[1]))

    # Load raster occupancy grid
    src = rasterio.open(rasterFile)
    grid = src.read(1)
    grid = np.flipud(grid)
    # Convert NaN to obstacles
    grid[np.isnan(grid)] = 1

    # Load points file
    manualPoints = None
    if pointsFile is not None:
        with open(pointsFile) as f:
                manualPoints = f.read().splitlines()

        manualPoints = [(int(point.split(',')[0]), int(point.split(',')[1])) for point in manualPoints]

    # Init roadmap
    prm = PRM(grid, 
              nSamples=nSamples,
              neighborhoodDist=neighborhoodDist,
              manualPoints=manualPoints)

    # Build roadmap
    if buildRoadmap:
        t0 = time.time()
        prm.learn()
        buildTime = time.time() - t0
        prm.save(pickleFile)
    else:
        prm.load(pickleFile)

    # Print roadmap info
    print("")
    print("Roadmap")
    print("-------")
    prm.description()
    if buildRoadmap:
        print("Build time: {} seconds".format(buildTime))

    # Plot map
    ax = prm.plot()
    ax.imshow(grid, cmap="Greys")

    if solvePath:
        # Solve path
        t0 = time.time()
        path, distance = prm.findPath(startCell, goalCell)
        solutionTime = time.time() - t0

        # Print path
        print("")
        print("Solution")
        print("--------")
        print(path)
        print("Path distance: {}".format(distance))
        print("Solution time: {} seconds".format(solutionTime))
        
        # Plot path
        rows, cols = zip(*path)
        plt.scatter(startCell[1], startCell[0], s=75, color="tab:orange")
        plt.scatter(goalCell[1], goalCell[0], s=75, color="tab:blue")
        plt.plot(cols, rows)

    plt.gca().invert_yaxis()
    if mapFile is not None:
        plt.savefig(mapFile)
    else:
        plt.show()

if __name__ == "__main__":
    main()


