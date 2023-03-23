import numpy as np
import scipy.interpolate as spi
import matplotlib.pyplot as plt

import platform
import math as m
import matplotlib.pyplot as plt
from zmqRemoteApi import RemoteAPIClient
import time

print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')

motorL=sim.getObject("/PioneerP3DX/leftMotor")
motorR=sim.getObject("/PioneerP3DX/rightMotor")
robot = sim.getObject("/PioneerP3DX")
sensors = [sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]') for i in range(16)]
#obstacles = [sim.getObject(f'/Cylinder[{i}]') for i in range(4)]
#obstaculos
obstacles = sim.createCollection(0);
sim.addItemToCollection(obstacles,sim.handle_all,-1,0)
sim.addItemToCollection(obstacles,sim.handle_tree,robot,1)

#detección 
noDetectionDist=0.5
maxDetectionDist=0.2
detect = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
braitenbergL = [-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
braitenbergR = [-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
v0=2

sim.startSimulation()

while sim.getSimulationTime() < 60:
    #ESQUIVAR
    for i in range(16):
        res, dist, _, _, _ = sim.readProximitySensor(sensors[i])
        if (res > 0 and dist < noDetectionDist):
            if (dist < maxDetectionDist):
                dist = maxDetectionDist
            detect[i]= 1 - ((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
        else:
            detect[i] = 0
        
    vLeft=v0
    vRight=v0

    for i in range(16):
        vLeft = vLeft + braitenbergL[i] * detect[i]
        vRight = vRight + braitenbergR[i] * detect[i]

    sim.setJointTargetVelocity(motorL,vLeft)
    sim.setJointTargetVelocity(motorR,vRight)

sim.stopSimulation()

time.sleep(1)


