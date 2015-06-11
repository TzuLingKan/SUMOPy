# -*- coding: utf-8 -*-
"""
Main file for running SUMO simulations with Python input.
Currently controls vehicle speed only.
last modified 6/11/15

Based off of:
@file    vehicleControl.py
@author  Michael Behrisch
@author  Daniel Krajzewicz
@author  Lena Kalleske
@date    2008-07-21
@version $Id: vehicleControl.py 16253 2014-04-25 12:09:01Z behrisch $
"""
import subprocess, sys, os, math
from optparse import OptionParser

from constants import * # sys.path is modified here
import collisionCheck, Sensors, Controllers

import traci
import traci.constants as tc

''' put the name of the SUMO config file to use here '''
CONFIGNAME = "collide1"

''' set sensor class here         '''
sensorToUse = Sensors.IdealSensor

dataFromTraciState = [tc.VAR_POSITION, tc.VAR_ANGLE, tc.VAR_SPEED]


class Setting:
    collisionOccurred = False
    verbose = False
setting = Setting()

class Step:
    step = 0
step = Step()

vehicleSetParams = {}
controllers = {}


def init(iteration = 0):
    ## set up SUMO parameters
    optParser = OptionParser()
    optParser.add_option("-v", "--verbose", action="store_true", dest="verbose",
                         default=False, help="tell me what you are doing")
    optParser.add_option("-g", "--gui", action="store_true", dest="gui",
                         default=False, help="run with GUI")
    optParser.add_option("-c", "--config", type="string", dest="CONFIGNAME",
                         default=None, help="SUMO config to use")
    (options, args) = optParser.parse_args()
    
    completeCommand = [SUMO]
    if options.gui:
        completeCommand = [SUMOGUI]
    sumoConfig = "%s.sumocfg" % (CONFIGNAME)
    if options.CONFIGNAME is not None:
        sumoConfig = "%s.sumocfg" % (options.CONFIGNAME)
    completeCommand += ["-c", sumoConfig]
    if iteration > 0:
        completeCommand += ["--fcd-output","./Results/out%s.xml" % (iteration)]   
    
    sumoProcess = subprocess.Popen(completeCommand, stdout=sys.stdout,
                                    stderr=sys.stderr)
    traci.init(PORT, 4)
    traci.simulation.subscribe()
    
    setting.verbose = options.verbose
    try:
        while step.step < 200 and not setting.collisionOccurred:
            doStep()
    finally:
        if not setting.collisionOccurred:        
            print "made it to the end without a collision"
        traci.close()
        sumoProcess.wait()
        
        if iteration > 0:
            return  setting.collisionOccurred

def getFromSubscription(vehID, subs, varNames):
    ## extracts specified elements from subscription and returns
    ## them as a list.  Pre-processes the element when necessary.
    data = []
    data.append(vehID)
    for varName in varNames:
        if varName == tc.VAR_POSITION:
            data.append((subs[varName][0]))
            data.append(subs[varName][1])
        elif varName == tc.VAR_ANGLE:
            data.append(math.radians(subs[varName]))
        else:
            data.append(subs[varName])
    return data
    
def getController(vehID, params = None):
    # The vehicle type names in the .rou file determine the control type
    # npc = use default SUMO motion
    # Is there a cleverer way to convert these names to functions?
    vtype = traci.vehicle.getTypeID(vehID)
    if vtype == "BasicControl":
        return Controllers.BasicControl(vehID, params)
    if vtype == "NoControl":
        return Controllers.NoControl(vehID, params)
    if vtype == "npc":
        return None
    if vtype == "Braker":
        return Controllers.Braker(vehID, params)
    raise KeyError("no matching controller for type",vtype)

def getSetParams(vehID):
    # these are vehicle variables that will not change over time
    timeStep = traci.simulation.getDeltaT() / 1000.0;
    return [traci.vehicle.getLength(vehID),
            traci.vehicle.getWidth(vehID),
            traci.vehicle.getAccel(vehID) * timeStep,
            traci.vehicle.getDecel(vehID) * timeStep]

def doStep():
    step.step += 1
    if setting.verbose:
        print "step", step.step
    traci.simulationStep()
    # adding new vehicles, if any    
    departed = traci.simulation.getSubscriptionResults()[tc.VAR_DEPARTED_VEHICLES_IDS]    
    for v in departed:
        traci.vehicle.subscribe(v,dataFromTraciState)
        vehicleSetParams[v] = getSetParams(v)
        controllers[v] = getController(v, vehicleSetParams[v])
        if True:#controllers[v] is not None:
            if setting.verbose:
                print "allowing forward and lane crashes for",v
            traci._sendIntCmd(tc.CMD_SET_VEHICLE_VARIABLE, tc.VAR_SPEEDSETMODE, v, 22)
    # gather vehicle states for this step
    vStates = {}
    for vehID, subs in traci.vehicle.getSubscriptionResults().iteritems():        
        tempParams = getFromSubscription(vehID, subs, dataFromTraciState)
        vStates[vehID] = VState( tempParams + vehicleSetParams[vehID] )
    #
    for vehID, vState in vStates.iteritems():
        sensor = sensorToUse(vState)
        #
        for otherID, otherState in vStates.iteritems():
            if vehID != otherID:
                #
                # check for collisions
                if collisionCheck.check(vState, otherState):
                    print "collision! pos",vState.x,vState.y,"step",step.step
                    setting.collisionOccurred = True
                    break
                #
                # update sensor
                sensor.addObstacle(otherState)
        #
        if setting.verbose:
            for vstat in sensor.getObstacles():
                print vehID, "detects", vstat.x, vstat.y,"speed",vstat.speed
        # use data to decide speed
        if controllers[vehID] is not None:
            controllers[vehID].updateSpeed(vState.speed)
            commandedSpeed = controllers[vehID].nextStep(sensor.getObstacles())
            traci.vehicle.setSpeed(vehID, commandedSpeed)
            if setting.verbose:
                print "setting speed", commandedSpeed


if __name__ == "__main__":
    init(0)