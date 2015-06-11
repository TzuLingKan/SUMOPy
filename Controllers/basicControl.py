# -*- coding: utf-8 -*-
"""
Simple example of collision-avoidance model.  Stores all permanent
characteristics upon construction, and takes its speed and the position of other objects
(relative to this one) every step.  Returns the speed for the next turn.
"""
import sys, os
sys.path.append(os.path.dirname(__file__)[:-len("/Controllers")])
from constants import * # sys.path is modified here
import math

class BasicControl:
    def __init__(self, vehID, vParams = None):
        self.ID = vehID
        self.speed = None
        self.obstacles = {}
        if vParams == None:
            self.length = 4
            self.width = 2
            self.maxAccel = 1
        else:
            self.length = vParams[0]
            self.width = vParams[1]
            self.maxAccel = vParams[2]
        
    def updateSpeed(self, speed):
        self.speed = speed
    
    def nextStep(self, obstacles):
        speed = self.speed + self.maxAccel
        for vstate in obstacles:
            self.obstacles[vstate.vehID] = vstate
            if vstate.vehID != self.ID:
                if 0 <= self.TTC(speed, vstate) < 3:
                    speed = self.speed
                if 0 <= self.TTC(speed, vstate) < 3:
                    speed = 0
        return speed
    
    def TTC(self, speed, obst):
        eps = .001      
        if (obst.angle + eps) % math.pi < 2*eps:
            if obst.speed == speed:
                return INFINITY
            if obst.y > 0: 
                return (obst.y - self.length)*1.0 / (obst.speed - speed)
            return (obst.y + obst.length)*1.0 / (obst.speed - speed)
        tCross = -obst.x / (obst.speed * math.sin(obst.angle))
        yMe = -tCross * speed
        yObstacle = obst.y - obst.speed * math.cos(obst.angle) * tCross
        minDistance = pow(pow(obst.length,2)+pow(obst.width,2)/4.0,.5) + self.length
        if abs(yMe - yObstacle) + eps < minDistance:
            return tCross
        return INFINITY
