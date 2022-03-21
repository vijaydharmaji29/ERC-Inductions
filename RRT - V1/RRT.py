import pygame
from RRTbasePy import RRTGraph
from RRTbasePy import RRTMap

def main():
    #initializing basic properties of the map
    dimensions = (600, 600)
    start = (50, 50)
    goal = (510, 510)
    obsdim = 30
    obsnum = 25
    iterations = 0

    pygame.init() #what does this do
    map = RRTMap(start, goal, dimensions, obsdim, obsnum) #map instance
    graph = RRTGraph(start, goal, dimensions, obsdim, obsnum) #graph instance

    obstacles = graph.makeobs() #using the graph instance to make obstacles, which will be passed onto the map to draw

    map.drawMap(obstacles)

    while not graph.path_to_goal(): #checks till goal is found
        if iterations % 10 == 0: #bias 10% of the time
            X, Y, Parent = graph.bias(goal)
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad, 0)
            pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), map.edgeThickness)
        else: #expand 90% of the time
            X, Y, Parent = graph.expand() #get x, y coordinate from expansion - that method takes care of checking and what not
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad, 0)
            pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), map.edgeThickness)

        if iterations % 5 == 0:
            pygame.display.update()

        iterations += 1

    map.drawPath(graph.getPathCoords())

    #pygame stuff to dipslay
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)


if __name__ == "__main__":
    main()