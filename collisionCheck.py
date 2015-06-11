# -*- coding: utf-8 -*-
"""
Checks for overlap between two vehicles
last modified 6/9/15
"""
import constants
from math import sin, cos, atan2

class Rectangle:
    
    def __init__(self, x, y, ang):
        self.x = x
        self.y = y
        self.ang = ang
        self.length = 4
        self.width = 2
    
    def __repr__(self):
        return "%s,%s,%s" % (self.x, self.y, self.ang) 
    
    def realign(self, other):
        # returns the shifted and rotated version of this rectangle
        # as if other were at the origin with angle 0
        tempx = self.x - other.x
        tempy = self.y - other.y
        tempAng = self.ang - other.ang
        relativeDist = pow((pow(tempx,2) + pow(tempy,2)),.5)
        relativeAng = atan2(tempy,tempx)
        rotationAng = relativeAng - other.ang
        return Rectangle(relativeDist * cos(rotationAng),
                         relativeDist * sin(rotationAng),
                         tempAng)
    
    def getCorners(self):
        # returns four corners of rectangle
        # assuming given coordinate is front-and-center of rectangle
        # and angle=0 is facing -y (this is SUMO's convention)
        x = self.x
        y = self.y
        ang = self.ang
        length = self.length
        width = self.width
        cornersX = [x + width/2 * cos(ang),
                x - width/2 * cos(ang),
                x - length*sin(ang) + width/2 * cos(ang),
                x - length*sin(ang) - width/2 * cos(ang)]
        cornersY = [y + width/2 * sin(ang),
                y - width/2 * sin(ang),
                y + length*cos(ang) + width/2 * sin(ang),
                y + length*cos(ang) - width/2 * sin(ang),]
        return [cornersX, cornersY]
    
    def outside(self,corners):
        # returns true if these corners are past the lines
        # that define this rectangle
        # if true, then definitely not overlapping
        allleft =  max(corners[0]) < -self.width/2
        allright = min(corners[0]) > self.width/2
        allabove = min(corners[1]) > self.length
        allbelow =  max(corners[1]) < 0
        return (allleft or allright or allabove or allbelow)
        
    def longestArc(self):
        return pow(pow(self.length,2) + pow(self.width,2)/4, .5)

def fromVState(vv):
    return Rectangle(vv.x, vv.y, vv.angle)

def checkRectangles(Rectangle1, Rectangle2):
    # fast check first
    longestConnection = Rectangle1.longestArc() + Rectangle2.longestArc()
    if abs(Rectangle1.x - Rectangle2.x) > longestConnection:
        return False
    if abs(Rectangle1.y - Rectangle2.y) > longestConnection:
        return False
    # now slow check
    view1 = Rectangle1.outside(Rectangle2.realign(Rectangle1).getCorners())
    view2 = Rectangle2.outside(Rectangle1.realign(Rectangle2).getCorners())
    return not (view1 or view2)
    
def check(vstate1, vstate2):
    rectangle1 = fromVState(vstate1)
    rectangle2 = fromVState(vstate2)
    return checkRectangles(rectangle1, rectangle2)
