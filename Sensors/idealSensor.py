#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Gives a vehicle complete knowledge about the others.
Note that it realigns them as if this vehicle has
position (0,0) and angle 0.
Last modified 6/11/15
"""
import sys, os
sys.path.append(os.path.dirname(__file__)[:-len("/Sensors")])
from constants import *
from math import cos, sin, atan2

class IdealSensor():
    
    def __init__(self,state):    
        self.state = state
        self.obstacles = []
    
    def addObstacle(self,vstate):
        if vstate.vehID == self.state.vehID:
            return
        # realign as if you are in origin, same as in collisionCheck
        tempx = vstate.x - self.state.x
        tempy = vstate.y - self.state.y
        newAngle = vstate.angle - self.state.angle
        relativeDist = pow((pow(tempx,2) + pow(tempy,2)),.5)
        relativeAngle = atan2(tempy,tempx)
        rotationAngle = relativeAngle - self.state.angle
        
        realignedState = vstate.copy()
        realignedState.x = relativeDist * cos(rotationAngle)
        realignedState.y = relativeDist * sin(rotationAngle)
        realignedState.angle = newAngle
        self.obstacles.append(realignedState)
    
    def getObstacles(self):
        return self.obstacles
