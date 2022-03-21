import random
import math
import pygame

class RRTMap:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        #basic variables required
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.Maph, self.Mapw = self.MapDimensions

        #pygame window settings
        self.MapWindowName = "RRT Path Planning"
        pygame.display.set_caption(self.MapWindowName)

        self.map = pygame.display.set_mode((self.Mapw, self.Maph))
        self.map.fill((255, 255, 255))

        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

        #storring obstacles
        self.obstacles = [] #obstacle would be stored in this as pygame.rect objects
        self.obsdim = obsdim
        self.obsNumber = obsnum

        #declaring some colors for ease of use
        self.grey = (70, 70, 70)
        self.Blue = (0, 0, 255)
        self.Green = (0, 255, 0)
        self.Red = (255, 0, 0)
        self.white = (255, 255, 255)

    def drawMap(self, obstacles):
        #drawing start and goal
        pygame.draw.circle(self.map, self.Green, self.start, self.nodeRad + 5, 0)
        pygame.draw.circle(self.map, self.Green, self.goal, self.nodeRad + 20, 1)

        #calling a draw obs method to draw obstacles
        self.drawObs(obstacles)

    def drawPath(self, path):
        for node in path:
            pygame.draw.circle(self.map, self.Red, node, self.nodeRad + 1, 0)

    def drawObs(self, obstacles):
        #drawing obstacles from obstacles passed through as argument which are a list of pygame.rect objects
        obstaclesList = obstacles.copy()

        while(len(obstaclesList) > 0):
            obstacle = obstaclesList.pop()
            pygame.draw.rect(self.map, self.grey, obstacle)


class RRTGraph:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        #basic required variables
        (x,y) = start

        self.start = start
        self.goal = goal
        self.goalFlag = False #will become true if the goal is reached
        self.maph, self.mapw = MapDimensions

        self.x = []
        self.y = []
        self.parent = []

        #adding start node | initializing the tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)

        #making note of obstacles
        self.obstacles = [] #list of pygame.rect objects
        self.obsDim = obsdim
        self.obsNum = obsnum

        #path
        self.goalstate = None
        self.path = []


    #getting random location for obstacle
    def makeRandomRect(self):
        uppercornerx = int(random.uniform(0, self.mapw -self.obsDim))
        uppercornery = int(random.uniform(0, self.maph - self.obsDim))

        return (uppercornerx, uppercornery)

    #using above function to create the rectangle object and adding it to the obstacles list
    def makeobs(self):
        obs = []

        for i in range(self.obsNum):
            rectang = None
            startgoalcol = True

            while startgoalcol:
                upper = self.makeRandomRect()

                rectang = pygame.Rect(upper, (self.obsDim, self.obsDim))

                if rectang.collidepoint(self.start) or rectang.collidepoint(self.goal):
                    startgoalcol = True
                else:
                    startgoalcol = False

            obs.append(rectang)

        self.obstacles = obs.copy()
        return obs


    def add_node(self, n, x, y):
        self.x.insert(n, x)
        self.y.insert(n, y)

    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def remove_egde(self, child):
        self.parent.pop(child)

    def number_of_nodes(self):
        return len(self.x)

    def distance(self, n1, n2):
        x1, y1 = self.x[n1], self.y[n1]
        x2, y2 = self.x[n2], self.y[n2]

        d = ((x1 - x2)**2 + (y1 - y2)**2)**0.5
        return d

    def sample_envir(self):
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))

        return x, y

    def nearest(self, n):
        dmin = self.distance(0, n)
        nnear = 0

        for i in range(n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i

        return nnear


    def isFree(self):
        n = self.number_of_nodes() - 1
        (x, y) = (self.x[n], self.y[n])

        obs = self.obstacles.copy()

        while len(obs) > 0:
            rectang = obs.pop()
            if rectang.collidepoint(x, y):
                self.remove_node(n)
                return False

        return True

    def crossObstacle(self, x1, x2, y1, y2):
        obs = self.obstacles.copy()

        while len(obs) > 0:
            rectang = obs.pop()
            for i in range(0, 101):
                u = i/100
                x = x1*u + x2*(1 - u)
                y = y1*u + y2*(1 - u)

                if rectang.collidepoint(x, y):
                    return True

        return False

    def connect(self, n1, n2):
        x1, y1 = self.x[n1], self.y[n1]
        x2, y2 = self.x[n2], self.y[n2]

        if self.crossObstacle(x1, x2, y1, y2):
            self.remove_node(n2)
            return False
        self.add_edge(n1, n2)
        return True


    def step(self, nnear, nrand, dmax=35):
        d = self.distance(nnear, nrand)

        if d > dmax:
            u = dmax/d
            xnear, ynear = self.x[nnear], self.y[nnear]
            xrand, yrand = self.x[nrand], self.y[nrand]

            px, py = (xrand - xnear, yrand - ynear)
            theta = math.atan2(py, px)

            x,y = int(xnear +dmax*math.cos(theta)), int(ynear +dmax*math.sin(theta))

            self.remove_node(nrand)

            if abs(x - self.goal[0]) < dmax and abs(y - self.goal[1]) < dmax:
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.add_node(nrand, x, y)

    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]

            while newpos != 0:
                self.path.append(newpos)
                newpos = self.parent[newpos]

            self.path.append(0)

        return self.goalFlag

    def getPathCoords(self):
        pathCoords = []

        for node in self.path:
            x, y = self.x[node], self.y[node]
            pathCoords.append((x, y))

        return pathCoords

    def bias(self, ngoal):
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)

        self.step(nnear, n)
        self.connect(nnear, n)

        return self.x, self.y, self.parent

    def expand(self):
        n = self.number_of_nodes() #getting index of new node
        x,y = self.sample_envir() #getting a new node

        self.add_node(n, x, y) #adding the new node

        if self.isFree(): #checking if the new node is in free space
            xnearest = self.nearest(n) #getting the nearest node to the new node
            self.step(xnearest, n) #checks if the distance is more the dmax
            self.connect(xnearest, n) #connects the 2 nodes

        return self.x, self.y, self.parent

    def cost(self):
        pass