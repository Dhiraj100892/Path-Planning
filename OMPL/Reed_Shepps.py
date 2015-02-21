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

def isStateValid(state):
    x = state.getX()
    y = state.getY()
    if x > 3 and x < 6 and ( (y > 0 and y < 2) or (y > 4 and y < 6) ):
        return 0
    else:
        return 1 
    
def planWithSimpleSetup():
    space = ob.ReedsSheppStateSpace(0.4)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(6)
    
    space.setBounds(bounds)

    start = ob.State(space)
    start().setX(0)
    start().setY(5)
    start().setYaw(math.pi/2)

    goal = ob.State(space)
    goal().setX(5)
    goal().setY(3)
    goal().setYaw(math.pi/2)

   
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    ss.setStartAndGoalStates(start, goal)
    si = ss.getSpaceInformation()
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

    points = 50
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
    
    
    rect1 = plt.Rectangle((3,0),3,2,fc='g')
    rect2 = plt.Rectangle((3,4),3,2,fc='g')
    plt.gca().add_patch(rect1)
    plt.gca().add_patch(rect2)

    plt.axis('equal')
    plt.grid(True)
    plt.axis([0,6,0,6])
    plt.show()
