# -*- coding: utf-8 -*-
"""
Simple example of collision-avoidance model.  Stores all permanent
characteristics upon construction, and takes its speed and the position of other objects
(relative to this one) every step.  Returns the speed for the next turn.
"""
import sys, os
sys.path.append(os.path.dirname(__file__)[:-len("/Controllers")])
from constants import * # sys.path is modified here

class NoControl:
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
        return self.speed + self.maxAccel