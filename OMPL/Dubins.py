from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
from math import sin, cos, tan
from functools import partial
import math
import time
try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import control as oc
    from ompl import geometric as og
except:
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))),'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import control as oc
    from ompl import geometric as og
    
''' center and radius for central obstacle'''
radius = 1
cX = 2
cY = 3

''' state validator function'''
def isStateValid(state):
    if math.sqrt( math.pow(state.getX()-cX,2) + math.pow(state.getY()-cY,2) ) > math.pow(radius,2):
        return 1
    else:
        return 0

''' Using ompl.geometric.SimpleSetup'''    
def planWithSimpleSetup():
    space = ob.ReedsSheppStateSpace(0.4)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(6)
    
    space.setBounds(bounds)

    #initial position
    start = ob.State(space)
    start().setX(0)
    start().setY(3)
    start().setYaw(math.pi)

    #goal poistion
    goal = ob.State(space)
    goal().setX(5)
    goal().setY(3)
    goal().setYaw(math.pi/2)

   
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    ss.setStartAndGoalStates(start, goal)
    si = ss.getSpaceInformation()
    
    #Selection of planner
    planner = og.RRTstar(si)
    ss.setPlanner(planner)
    ss.setOptimizationObjective(ob.MechanicalWorkOptimizationObjective(si))
    solved = ss.solve(5)


    if solved:
        print(ss.getPlanner())
        ss.simplifySolution()
        data = ss.getSolutionPath().printAsMatrix()
        f = open('path.txt','w')
        f.write(data)
        f.close
        d = numpy.loadtxt('path.txt')

        #plot
        

    else:
        print("No Solution Found")
    
    plt.show()
    return space
if __name__ == '__main__':
    space = planWithSimpleSetup()
    
    d = numpy.loadtxt('path.txt')
    print('read')
    print(d)

    points = 20
    x = numpy.zeros(shape=(points+1,1))
    y = numpy.zeros(shape=(points+1,1))
    S = ob.State(space)
    G = ob.State(space)
    st = ob.State(space)
    t = 1.0/points
    for j in range(0,numpy.size(d[:,1]) - 1):
        S().setX(d[j,0])
        S().setY(d[j,1])
        S().setYaw(d[j,2])

        G().setX(d[j+1,0])
        G().setY(d[j+1,1])
        G().setYaw(d[j+1,2])
        for i in range(0,points+1):
            space.interpolate(S(),G(),t*i,st())
            x[i] = st().getX()
            y[i] = st().getY()

        plt.plot(x,y,'green')
    
    
    circle  = plt.Circle((cX,cY),radius,fc = 'y')
    plt.gca().add_patch(circle)
    
    plt.axis('equal')
    plt.grid(True)
    plt.axis([0,6,0,6])
    plt.show()
