#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Occupancy grid creation using a Pioneer pd3x with ultrasonic sensors.

Author: Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
Mobile Robotics course, University of Guanajuato (2023)
"""

import numpy as np
import time
import math as m
import matplotlib.pyplot as plt
import os
from zmqRemoteApi import RemoteAPIClient
import cv2

def q2R(x,y,z,w):
    R = np.zeros((3,3))
    R[0,0] = 1-2*(y**2+z**2)
    R[0,1] = 2*(x*y-z*w)
    R[0,2] = 2*(x*z+y*w)
    R[1,0] = 2*(x*y+z*w)
    R[1,1] = 1-2*(x**2+z**2)
    R[1,2] = 2*(y*z-x*w)
    R[2,0] = 2*(x*z-y*w)
    R[2,1] = 2*(y*z+x*w)
    R[2,2] = 1-2*(x**2+y**2)
    return R

def grid_cells(x, y):
    xr = center_x + m.ceil((x*multi)/scale)
    yr = center_y - m.floor((y*multi)/scale)
    if xr >= width:
        xr = width
    elif xr < 0:
        xr = 0
    if yr >= height:
        yr = height
    elif yr < 0:
        yr = 0
    return xr, yr

def check_position(x, y):
    global occgrid, tocc, width, height, center_x, center_y, flags
    # MATRIZ 1 * 2
    if flags[0] == 0 and x > 1.5:        
        print("MATRIZ 1 * 2")
        occgrid = np.hstack((occgrid, 0.5*np.ones((100,100))))
        tocc = np.hstack((tocc, np.zeros((100,100))))
        width = 200        
        flags[0] = 1        
    # MATRIZ 2 * 2
    elif flags[1] == 0  and y > 2.5:
        print("MATRIZ 2 * 2")        
        occgrid = np.vstack((0.5*np.ones((100,200)), occgrid))     
        tocc = np.vstack((np.zeros((100,200)), tocc))
        height = 200
        center_y = 150
        flags[1] = 1        
    # MATRIZ 2 * 3
    elif flags[2] == 0 and x < -1.5:
        print("MATRIZ 2 * 3")
        occgrid = np.hstack((0.5*np.ones((200,100)), occgrid))
        tocc = np.hstack(( np.zeros((200,100)), tocc))        
        width = 300
        center_x = 150
        flags[2] = 1
    # MATRIZ 3 * 3
    elif flags[3] == 0 and y < -2.5:  
        print("MATRIZ 3 * 3")      
        occgrid = np.vstack((occgrid, 0.5*np.ones((100,300))))
        tocc = np.vstack((tocc, np.zeros((100,300))))
        height = 300
        flags[3] = 1        

# def check_sensors(i):
#     global uread, upt, ustate, usensor, Rs, Vs, tocc
#     state, distance, point, detectedObj, _ = sim.readProximitySensor(usensor[i])
        
#     uread.append(distance)
#     upt.append(point)
#     ustate.append(state)        
#     # Transform detection from sensor frame to robot frame
#     if state == True:
#         opos = np.array(point).reshape((3,1))
#     else:
#         opos = np.array([0,0,1]).reshape((3,1))
#     robs = np.matmul(Rs[i], opos) + Vs[i]
        
#     # Transform detection from robot frame to global frame
        
#     R = q2R(carrot[0], carrot[1], carrot[2], carrot[3])
#     rpos = np.array(carpos).reshape((3,1))
#     pobs = np.matmul(R, robs) + rpos        

#     xo, yo = grid_cells(pobs[0], pobs[1]) 
#     # SI SE DETECTA ALGO SE REGISTRA OBSTÁCULO       
#     if state:
#         tocc[xo-1, yo-1] = 1
#     return xo, yo

client = RemoteAPIClient()
sim = client.getObject('sim')

motorL=sim.getObject('/PioneerP3DX/leftMotor')
motorR=sim.getObject('/PioneerP3DX/rightMotor')
robot = sim.getObject('/PioneerP3DX')

sim.startSimulation()

# Assigning handles to the ultrasonic sensors
usensor = []
Rs = []
Vs = []
for i in range(0,16):
    s = sim.getObject('/PioneerP3DX/ultrasonicSensor['+str(i)+']')
    usensor.append(s)
    q = sim.getObjectQuaternion(s, robot)
    Rs.append(q2R(q[0], q[1], q[2], q[3]))
    Vs.append(np.reshape(sim.getObjectPosition(s, robot), (3,1)))

carpos = sim.getObjectPosition(robot, -1)
carrot = sim.getObjectOrientation(robot, -1)

Kv = 0.5
Kh = 2.5
xd = 3
yd = 3
hd = 0
r = 0.1
L = 0.2
errp = 10

#BANDERAS
flags = [0, 0, 0, 0]

if os.path.exists('map.txt'):
    print('Map found. Loading...')
    occgrid = np.loadtxt('map.txt')
    tocc = 1.0*(occgrid > 0.5)
    occgrid[occgrid > 0.5] = 0
    width = 300
    height = 300
    flags[3] = 1
    # CENTROS PARA LA TRANSFORMACIÓN
    center_x = 150
    center_y = 150
else:
    print('Creating new map')
    occgrid = 0.5*np.ones((100,100))
    tocc = np.zeros((100,100))
    height = 100
    width = 100
    # CENTROS PARA LA TRANSFORMACIÓN
    center_x = 50
    center_y = 50

t = time.time()

initt = t
niter = 0

#MATRIZ Y CONSTANTES
scale = 0.1
multi = 2

while time.time()-t < 30:
    carpos = sim.getObjectPosition(robot, -1)
    
    # xw = carpos[0]
    # yw = carpos[1]

    if flags[3] != 1:
        check_position(carpos[0], carpos[1])
        # check_position(xw, yw)

    xr, yr = grid_cells(carpos[0], carpos[1])
    occgrid[yr-1, xr-1] = 0

    carrot = sim.getObjectQuaternion(robot, -1)

    uread = []
    ustate = []
    upt = []
    
    for i in range(0,16, 2):
        state, distance, point, detectedObj, _ = sim.readProximitySensor(usensor[i])
        
        uread.append(distance)
        upt.append(point)
        ustate.append(state)
        
        # Transform detection from sensor frame to robot frame
        if state == True:
            opos = np.array(point).reshape((3,1))
        else:
            opos = np.array([0,0,1]).reshape((3,1))
        robs = np.matmul(Rs[i], opos) + Vs[i]
        
        # Transform detection from robot frame to global frame
        
        R = q2R(carrot[0], carrot[1], carrot[2], carrot[3])
        rpos = np.array(carpos).reshape((3,1))
        pobs = np.matmul(R, robs) + rpos        

        # Transform detection from global frame to occupancy grid cells
        xs = pobs[0]
        ys = pobs[1]

        xo, yo = grid_cells(xs, ys) 
        # SI SE DETECTA ALGO SE REGISTRA OBSTÁCULO       
        if state:
            tocc[yo-1, xo-1] = 1
        # SE TRAZA LINEA HACIA EL OBSTÁCULO
        occgrid = cv2.line(occgrid, (xr-1, yr-1), (xo-1, yo-1), (0,0,0), 1)

    # Reactive navigation block
    ul = 1
    ur = 1
    lgains = np.linspace(0,-1,len(upt)//2)
    rgains = np.linspace(-1,0,len(upt)//2)
    for k in range(len(upt)//2):
        if ustate[k]:
            ul = ul + lgains[k]*(1.0 - uread[k])
            ur = ur + rgains[k]*(1.0 - uread[k])    

    sim.setJointTargetVelocity(motorL, ul)
    sim.setJointTargetVelocity(motorR, ur)

    niter = niter + 1

print(lgains)
print(rgains)
finalt = time.time()
# print('Avg time per iteration ', (finalt-initt)/niter)

sim.setJointTargetVelocity(motorL, 0)
sim.setJointTargetVelocity(motorR, 0)
    
sim.stopSimulation()

plt.imshow(tocc+occgrid)
plt.show()
np.savetxt('map.txt', tocc+occgrid)