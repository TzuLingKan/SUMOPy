# -*- coding: utf-8 -*-
"""
Simple example of collision-avoidance model.  Stores all permanent
characteristics upon construction, and takes its speed and the position of other objects
(relative to this one) every step.  Returns the speed for the next turn.
"""
import sys, os
sys.path.append(os.path.dirname(__file__)[:-len("/Controllers")])
from constants import * # sys.path is modified here
import random


class Braker:
    def __init__(self, vehID, vParams = None):
        self.ID = vehID
        self.speed = None
        self.obstacles = {}
        if vParams == None:
            self.length = 4
            self.width = 2
            self.maxAccel = 1
            self.maxDecel = 1
        else:
            self.length = vParams[0]
            self.width = vParams[1]
            self.maxAccel = vParams[2]
            self.maxDecel = vParams[3]
        self.braking = []
        
    def updateSpeed(self, speed):
        self.speed = speed
    
    def nextStep(self, obstacles):
        if len(self.braking) == 0:
            if self.probabilityToBrake():
                self.braking = self.brakingScenario();
            return -1
        
        return self.speed - self.braking.pop()
    
    
    def probabilityToBrake(self):
        # 1 in n chance of braking  
        n = 20
        return random.randint(1,n) == 1
    
    
    def brakingScenario(self):
        # uniformly choose ending speed and drop rate
        totalDecrease = random.random() * self.speed
        stepDecrease = random.random() * (self.maxDecel - .25) + .25
        length = int(totalDecrease / stepDecrease + .5)
        return [stepDecrease] * length