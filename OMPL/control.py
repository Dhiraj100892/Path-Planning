
#!/usr/bin/env python
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
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))),'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import control as oc
    from ompl import geometric as og

radius = 5
cX = 30
cY = 30

# state validator function
def isStateValid(state):
    
    if math.sqrt( math.pow(state.getX()-cX,2) + math.pow(state.getY()-cY,2) ) > math.pow(radius,2):
        return 1
    else:
        return 0

# progator function based on the kinematics of the car(steering rear wheel distnce is considered 1 )   
def propagate(start, control, duration, state):
    state.setX( start.getX() + control[0] * duration * cos(start.getYaw()) )
    state.setY( start.getY() + control[0] * duration * sin(start.getYaw()) )
    state.setYaw(start.getYaw() + control[0] * duration * tan(control[1]) )

# Using ompl.control.SimpleSetup
def planWithSimpleSetup():
    space = ob.SE2StateSpace()
    
    # define the bounds
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(100)
    
    space.setBounds(bounds)
    
    #set start and and goal
    start = ob.State(space)
    start().setX(10)
    start().setY(10)
    start().setYaw(mayh.pi/2)

    goal = ob.State(space)
    goal().setX(50)
    goal().setY(50)
    goal().setYaw(0)
    
    # control bounds
    cspace = oc.RealVectorControlSpace(space,2)
    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(-0.3)
    cbounds.setHigh(0.3)
    cspace.setBounds(cbounds)
    
    # using SimpleSeup class in ompl.control
    ss = oc.SimpleSetup(cspace)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    ss.setStatePropagator(oc.StatePropagatorFn(propagate))
    
    si = ss.getSpaceInformation()

    ss.setOptimizationObjective(ob.MechanicalWorkOptimizationObjective(si))

    #set the planner
    planner = oc.RRT(si)
    ss.setPlanner(planner)

    ss.setStartAndGoalStates(start, goal)
    solved = ss.solve(40)
    if solved:
        print(ss.getPlanner())
        data = ss.getSolutionPath().asGeometric().printAsMatrix()
        print('data')
        print(data)
        f = open('path.txt','w')
        f.write(data)
        f.close
        
    else:
        print("No Solution Found")
    
    
if __name__ == '__main__':
    planWithSimpleSetup()
    data = numpy.loadtxt('path.txt')
    
    #for visualization
    plt.plot(data[:,0],data[:,1],'.-')
    circle  = plt.Circle((cX,cY),radius,fc = 'y')
    plt.gca().add_patch(circle)
    plt.grid(True)
    plt.show()

    
    
    
    
    
    

