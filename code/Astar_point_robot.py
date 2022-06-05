import pygame
import math
import heapq
import time
import functools

# Defining Graph Constants
HEIGHT = 500
WIDTH = 500
BLACK = (255, 255, 255)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
MAGENTA = (255,0,0)
WHITE= (0, 0, 0)


class Node:
    """
    Node class : This class is built to store the node information.
    A node is simply a location on a map. For each node, its neighbours, parents & distance to reach that node is stored.
    """

    def __init__(self, i, j, endI, endJ, theta):
        """
        Description: Defining all properties for each node - Neighbours, Parents, Distance.
        """
        self.i = i
        self.j = j
        self.theta = theta
        self.costToCome = 0.0
        self.costToGo = 2.5*(math.sqrt((i - endI) ** 2 + (j - endJ) ** 2))
        self.cost = None
        self.neighbours = {}
        self.valid_actions = {}
        self.parent = None

    def __lt__(self, other):
        return self.cost < other.cost

class Graph:
    """
    Graph class : This class defines all methods to generate a graph and perform AStar Algorithm.
    """

    def __init__(self, start, end, RPM1, RPM2, RADIUS, CLEARANCE):
        self.visited = {}
        self.endI = end.i
        self.endJ = end.j
        self.RPM1 = RPM1
        self.RPM2 = RPM2
        self.RADIUS = RADIUS
        self.CLEARANCE = CLEARANCE + self.RADIUS

    def getNeighbours(self, currentNode):
        """
        Description: Returns neighbours for the currentNode.
        """
        i, j, theta = currentNode.i, currentNode.j, currentNode.theta
        neighbours = {}
        valid_actions = {}
        actions = [[0, self.RPM1], [self.RPM1, 0], [self.RPM1, self.RPM1], [0, self.RPM2], [self.RPM2, 0], [self.RPM2, self.RPM2], [self.RPM1, self.RPM2], [self.RPM2, self.RPM1]]
        for UL, UR in actions:
            x, y, newTheta, distance = self.getNewCoordiantes(i, j, theta, UL, UR)
            if (not self.isOutsideArena(x, y)) and (not self.isAnObstacle(x, y)):
                newNode = Node(x, y, self.endI, self.endJ, newTheta)
                neighbours[newNode] = distance
                valid_actions[newNode] = [UL, UR]
        return neighbours, valid_actions

    def getNewCoordiantes(self, i, j, theta, UL, UR):
        t = 0
        r = 0.033
        L = 0.16
        dt = 0.01

        UL = 3.14*UL/30
        UR = 3.14*UR/30

        newI = i
        newJ = j
        newTheta = 3.14 * theta/180
        D = 0

        while t < 1:
            t = t + dt
            Delta_Xn = 0.5 * r * (UL + UR) * math.cos(newTheta) * dt
            Delta_Yn = 0.5 * r * (UL + UR) * math.sin(newTheta) * dt
            newI += Delta_Xn
            newJ += Delta_Yn
            newTheta += (r / L) * (UR - UL) * dt
            D = D + math.sqrt(math.pow(Delta_Xn, 2) + math.pow(Delta_Yn, 2))
        newTheta = 180*newTheta/3.14

        if newTheta > 0:
            newTheta = newTheta % 360
        elif newTheta < 0:
            newTheta = (newTheta + 360) % 360

        newI = self.getRoundNumber(newI)
        newJ = self.getRoundNumber(newJ)

        return newI, newJ, newTheta, D

    def draw_action(self, i, j, theta, UL, UR, color):
        t = 0
        r = 0.033
        L = 0.16
        dt = 0.01

        newI = i
        newJ = j
        newTheta = 3.14*theta/180
        UL = 3.14*UL/30
        UR = 3.14*UR/30

        while t < 1:
            t = t + dt
            oldI = newI
            oldJ = newJ
            newI += 0.5 * r * (UL + UR) * math.cos(newTheta) * dt
            newJ += 0.5 * r * (UL + UR) * math.sin(newTheta) * dt
            pygame.draw.line(gridDisplay, color, [int(50*oldI), int(HEIGHT - 50*oldJ)], [int(50*newI), int(HEIGHT - 50*newJ)], 2)
            newTheta += (r / L) * (UR - UL) * dt
        pygame.display.update()
        time.sleep(0.1)

    def getRoundNumber(self, i):

        i = 50*i
        i = int(i)
        i = i/50
        return i
    
    def generateGraph(self, ):
        """
        Description: Checks if a point is in the Ellipse.
        Input: Point with co-ordinates (x,y)
        Output: True or False
        """

        # Make background White
        gridDisplay.fill(WHITE)

        # Circles
        pygame.draw.circle(gridDisplay, MAGENTA, [100, int(HEIGHT - 100)], 50)
        pygame.draw.circle(gridDisplay, MAGENTA, [100, int(HEIGHT - 400)], 50)

        # Rectangles
        pygame.draw.polygon(gridDisplay, MAGENTA, [(int(50*0.25), int(HEIGHT - 50*5.75)), (int(50*1.75), int(HEIGHT - 50*5.75)), (int(50*1.75), int(HEIGHT - 50*4.25)), (50*0.25, HEIGHT - 50*4.25)])
        pygame.draw.polygon(gridDisplay, MAGENTA, [(int(50*3.75), int(HEIGHT - 50*5.75)), (int(50*6.25), int(HEIGHT - 50*5.75)), (int(50*6.25), int(HEIGHT) - int(50*4.25)), (int(50*3.75), int(HEIGHT - 50*4.25))])
        pygame.draw.polygon(gridDisplay, MAGENTA, [(int(50*7.25), int(HEIGHT - 50*4)), (int(50*8.75), int(HEIGHT - 50*4)), (int(50*8.75), int(HEIGHT - 50*2)), (int(50*7.25), int(HEIGHT - 50*2))])

    def performAStar(self, start, end):
        """
        Description: Defining initial constants - Visited array, Rows, Cols, Target String.
        Input: Starting and ending node for the robot to browse.
        Output: Returns True or False to define if an optimal path can be found or not.
        """

        # Checking is start and end are in obstancle.
        if self.isAnObstacle(start.i, start.j) and self.isAnObstacle(end.i, end.j):
            print("Starting and ending point are inside the obstacle!")
            return

        if self.isAnObstacle(start.i, start.j):
            print("Starting point is inside the obstacle!")
            return
        if self.isAnObstacle(end.i, end.j):
            print("Ending point is inside the obstacle!")
            return

        if self.isOutsideArena(start.i, start.j):
            print("Starting point is outside the arena!")
            return

        if self.isOutsideArena(end.i, end.j):
            print("Ending point is outside the arena!")
            return

        print("Finding path...")
        priorityQueue = []
        visited_list = {}
        heapq.heappush(priorityQueue, (start.cost, start))
        while len(priorityQueue):
            currentNode = heapq.heappop(priorityQueue)
            currentNode = currentNode[1]
            if self.isInsideTargetArea(currentNode.i, currentNode.j):
                print("Found a path!")
                return True

            if tuple([currentNode.i, currentNode.j]) in visited_list:
                continue
            visited_list[tuple([currentNode.i, currentNode.j])] = True

            currentDistance = currentNode.costToCome
            neighbours, valid_actions = self.getNeighbours(currentNode)
            currentNode.neighbours = neighbours
            currentNode.valid_actions = valid_actions
            for neighbourNode, newDistance in neighbours.items():
                neighbourNode.costToCome = currentDistance + newDistance
                neighbourNode.cost = neighbourNode.costToCome + neighbourNode.costToGo
                neighbourNode.parent = currentNode
                heapq.heappush(priorityQueue, (neighbourNode.cost, neighbourNode))
                print((neighbourNode.i, neighbourNode.j))
        print("Cannot find a path :(")
        return False

    def visualizeAStar(self, start, end):
        """
        Description: Visualization of the algorithm.
        Input: Starting and ending node for the robot to browse.
        Output: A animation of nodes which are browsed and the path generated.
        """

        visited_list = {}
        priorityQueue = []
        heapq.heappush(priorityQueue, (start.cost, start))
        pygame.draw.circle(gridDisplay, BLACK, [int(50*start.i), int(HEIGHT - 50*start.j)], 5)
        pygame.draw.circle(gridDisplay, BLACK, [int(50*end.i), int(HEIGHT - 50*end.j)], 5)
        pygame.display.update()
        while len(priorityQueue):

            currentNode = heapq.heappop(priorityQueue)
            currentNode = currentNode[1]

            if self.isInsideTargetArea(currentNode.i, currentNode.j):
                self.backTrack(currentNode)
                print("Distance Required to reach from start to end is:", currentNode.costToCome)
                return

            if tuple([currentNode.i, currentNode.j]) in visited_list:
                continue
            visited_list[tuple([currentNode.i, currentNode.j])] = True

            for neighbourNode, action in currentNode.valid_actions.items():
                self.draw_action(currentNode.i,currentNode.j,currentNode.theta,action[0],action[1],CYAN)

            for neighbourNode, newDistance in currentNode.neighbours.items():
                heapq.heappush(priorityQueue, (neighbourNode.cost, neighbourNode))

    def backTrack(self, child):
        """
        Description: Backtracking from the finishing node to the start node.
        Input: Ending Node
        Output: A animation of the path generated.
        """
        while child != None:
            path.append(child)
            print(child.i, child.j, "Path")
            child = child.parent
        return True

    def isinStartingCircle(self, start, i, j):
        """
        Description: Checks if a point is in the starting circle from where the robot will start,
        Input: Point with co-ordinates (x,y)
        Output: True or False
        """

        if (i - start.i) ** 2 + (j - start.j) ** 2 - 1 <= 0:
            return True
        else:
            return False

    def isInStartingSquare(self, start, i, j):
        """
        Description: Checks if a point is in the starting square from where the robot will start.
        Input: Point with co-ordinates (x,y)
        Output: True or False
        """

        if (start.i - 0.5 <= i <= start.i + 0.5) and (start.j - 0.5 <= j <= start.j + 0.5):
            return True
        else:
            return False

    def isInCircle1(self, x, y):
        """
        Description: Checks if a point is in the circle.
        Input: Point with co-ordinates (x,y)
        Output: True or False
        """
        r = 1 + self.CLEARANCE
        if (x - 2) ** 2 + (y - 2) ** 2 - r ** 2 >= 0:
            return False
        else:
            return True

    def isInCircle2(self, x, y):
        """
        Description: Checks if a point is in the circle.
        Input: Point with co-ordinates (x,y)
        Output: True or False
        """
        r = 1 + self.CLEARANCE
        if (x - 2) ** 2 + (y - 8) ** 2 - r ** 2 >= 0:
            return False
        else:
            return True

    def isInRectangle1(self, x, y):
        """
        Description: Checks if a point is in the rotated rectangle.
        Input: Point with co-ordinates (x,y)
        Output: True or False
        """
        circ1 = (x - 0.25) ** 2 + (y - 5.75) ** 2 <= self.CLEARANCE ** 2
        circ2 = (x - 1.75) ** 2 + (y - 5.75) ** 2 <= self.CLEARANCE ** 2
        circ3 = (x - 0.25) ** 2 + (y - 4.25) ** 2 <= self.CLEARANCE ** 2
        circ4 = (x - 1.75) ** 2 + (y - 4.25) ** 2 <= self.CLEARANCE ** 2
        side1 = x <= 1.75
        eside1 = x <= 1.75 + self.CLEARANCE
        side2 = y <= 5.75
        eside2 = y <= 5.75 + self.CLEARANCE
        side3 = x >= 0.25
        eside3 = x >= 0.25 - self.CLEARANCE
        side4 = y >= 4.25
        eside4 = y >= 4.25 - self.CLEARANCE
        rect1 = eside1 and side2 and eside3 and side4
        rect2 = side1 and eside2 and side3 and eside4

        if rect1 or rect2 or circ1 or circ2 or circ3 or circ4:
            return True
        else:
            return False

    def isInRectangle2(self, x, y):
        """
        Description: Checks if a point is in the rotated rectangle.
        Input: Point with co-ordinates (x,y)
        Output: True or False
        """
        circ1 = (x - 3.75) ** 2 + (y - 5.75) ** 2 <= self.CLEARANCE ** 2
        circ2 = (x - 6.25) ** 2 + (y - 5.75) ** 2 <= self.CLEARANCE ** 2
        circ3 = (x - 3.75) ** 2 + (y - 4.25) ** 2 <= self.CLEARANCE ** 2
        circ4 = (x - 6.25) ** 2 + (y - 4.25) ** 2 <= self.CLEARANCE ** 2
        side1 = x <= 6.25
        eside1 = x <= 6.25 + self.CLEARANCE
        side2 = y <= 5.75
        eside2 = y <= 5.75 + self.CLEARANCE
        side3 = x >= 3.75
        eside3 = x >= 3.75 - self.CLEARANCE
        side4 = y >= 4.25
        eside4 = y >= 4.25 - self.CLEARANCE
        rect1 = eside1 and side2 and eside3 and side4
        rect2 = side1 and eside2 and side3 and eside4

        if rect1 or rect2 or circ1 or circ2 or circ3 or circ4:
            return True
        else:
            return False

    def isInRectangle3(self, x, y):
        """
        Description: Checks if a point is in the rotated rectangle.
        Input: Point with co-ordinates (x,y)
        Output: True or False
        """
        circ1 = (x - 7.25) ** 2 + (y - 4) ** 2 <= self.CLEARANCE ** 2
        circ2 = (x - 8.75) ** 2 + (y - 4) ** 2 <= self.CLEARANCE ** 2
        circ3 = (x - 8.75) ** 2 + (y - 2) ** 2 <= self.CLEARANCE ** 2
        circ4 = (x - 7.25) ** 2 + (y - 2) ** 2 <= self.CLEARANCE ** 2
        side1 = x <= 8.75
        eside1 = x <= 8.75 + self.CLEARANCE
        side2 = y <= 4
        eside2 = y <= 4 + self.CLEARANCE
        side3 = x >= 7.25
        eside3 = x >= 7.25 - self.CLEARANCE
        side4 = y >= 2
        eside4 = y >= 2 - self.CLEARANCE
        rect1 = eside1 and side2 and eside3 and side4
        rect2 = side1 and eside2 and side3 and eside4

        if rect1 or rect2 or circ1 or circ2 or circ3 or circ4:
            return True
        else:
            return False

    def isAnObstacle(self, x, y):
        """
        Description: Checks if the point (x,y) is inside an obstacle or not.
        Input: Point with co-ordinates (x,y)
        Output: True or False
        """

        return self.isInCircle1(x, y) or self.isInCircle2(x, y) or self.isInRectangle1(x, y) or self.isInRectangle2(x, y) or self.isInRectangle3(x, y)

    def isOutsideArena(self, x, y):
        """
        Description: Checks if the point (x,y) is outside the areana or not.
        Input: Point with co-ordinates (x,y)
        Output: True or False
        """

        return True if x < self.CLEARANCE or y < self.CLEARANCE or x > 10 - self.CLEARANCE or y > 10 - self.CLEARANCE else False

x1 = float(input("Enter the x coordiante of the starting point: "))
y1 = float(input("Enter the y coordiante of the starting point: "))
thetaStart = int(input("Enter the start theta: "))
print("#############################################")

x2 = float(input("Enter the x coordiante of the ending point: "))
y2 = float(input("Enter the y coordiante of the ending point: "))
print("#############################################")

RPM1 = float(input("Enter RPM1: "))
RPM2 = float(input("Enter RPM2: "))
print("#############################################")
# RPM1 = 100.0
# RPM2 = 70.0
RADIUS = float(input("Enter the radius of the robot:  "))
CLEARANCE = float(input("Enter the clearance:  "))

#############################################
# Algorithm Driver
end = Node(x2, y2, x2, y2, 0)
start = Node(x1, y1, x2, y2, thetaStart)
start.costToCome = 0
robot = Graph(start, end, RPM1, RPM2, RADIUS, CLEARANCE)
path = []

# Check if path can be found
if robot.performAStar(start, end):
    pass
    pygame.init()  # Setup Pygame
    gridDisplay = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("A* Algorithm - Rigid Robot")
    exiting = False
    clock = pygame.time.Clock()
    grid = [[0 for j in range(HEIGHT)] for i in range(WIDTH)]
    canvas = Graph(start, end, RPM1, RPM2, RADIUS, CLEARANCE)  # Create Canvas
    canvas.generateGraph()
    robot.visualizeAStar(start, end)
    path.reverse()
else:
    # No Path Found
    exiting = True

#############################################
# Running the simulation in loop

while not exiting:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            exiting = True

            # Visualizing the final path
    for index in range(len(path)-1):
        node = path[index]
        action = node.valid_actions[path[index+1]]
        robot.draw_action(node.i, node.j, node.theta, action[0], action[1], MAGENTA)


    clock.tick(2000)
    pygame.display.flip()
    exiting = True
pygame.quit()
#############################################