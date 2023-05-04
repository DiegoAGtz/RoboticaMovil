# -*- coding: utf-8 -*-
"""
@author: DiegoAGtz

@description: Primer proyecto de la materia Robótica Móvil
"""
import math as m
import os
import platform
import random
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
import scipy.interpolate as spi
from zmqRemoteApi import RemoteAPIClient


def get_host():
    so = platform.system()
    if so != "Linux":
        return "localhost"

    ip = ""
    with open("/etc/resolv.conf", "r") as f:
        data = f.readlines()
        ip = data[-1][:-1]
        ip = ip[11:]
    return ip


def v2u(v, omega, r, L):
    ur = v / r + L * omega / (2 * r)
    ul = v / r - L * omega / (2 * r)
    return ur, ul


def angdiff(t1, t2):
    angmag = m.acos(m.cos(t1) * m.cos(t2) + m.sin(t1) * m.sin(t2))
    angdir = m.cos(t1) * m.sin(t2) - m.sin(t1) * m.cos(t2)
    return m.copysign(angmag, angdir)


def q2R(x, y, z, w):
    R = np.zeros((3, 3))
    R[0, 0] = 1 - 2 * (y**2 + z**2)
    R[0, 1] = 2 * (x * y - z * w)
    R[0, 2] = 2 * (x * z + y * w)
    R[1, 0] = 2 * (x * y + z * w)
    R[1, 1] = 1 - 2 * (x**2 + z**2)
    R[1, 2] = 2 * (y * z - x * w)
    R[2, 0] = 2 * (x * z - y * w)
    R[2, 1] = 2 * (y * z + x * w)
    R[2, 2] = 1 - 2 * (x**2 + y**2)
    return R

def calculateMap(center, scala, Map, obstacles, sensor):
    
    sSensor = [int(i/scala) for i in sensor[:2]] 
    cSensor = [center[0] - sSensor[1], center[1] + sSensor[0]]
    #cSensor = [center[0] - sSensor[0], center[1] + sSensor[1]]

    newMap = np.copy(Map)
    newObstacles = np.copy(obstacles)
    newCenter = center
    size = newMap.shape
    
    combineMap = lambda m1, m2, n: np.hstack((m1, m2)) if bool(n) else np.vstack((m1, m2))
    
    for i, _ in enumerate(size):
        if(cSensor[i] > size[i]):
            if(i):
                newSize = (size[i-1], int(cSensor[i] - size[i]))
            else:
                newSize = (int(cSensor[i] - size[i]), size[i+1])
            newMap = combineMap(np.copy(newMap), 0.5*np.ones(newSize), i)
            newObstacles = combineMap(np.copy(newObstacles), np.zeros(newSize), i)
            size = newMap.shape
        
        if(cSensor[i] < 0):
            if(i):
                newSize = (size[i-1], int(abs(cSensor[i])))
            else:
                newSize = (int(abs(cSensor[i])), size[i+1])
            newMap = combineMap(0.5*np.ones(newSize), np.copy(newMap), i)
            newObstacles = combineMap(np.zeros(newSize), np.copy(newObstacles), i)
            newCenter[i] += newSize[i]
            size = newMap.shape
    
    
    return [newMap, newObstacles, newCenter]
    


def readSensors(sim, sensors):
    max_distance = 0.8
    detect = False
    velocities = [0] * len(sensors)
    for i in range(len(sensors)):
        result, distance, _, _, _ = sim.readProximitySensor(sensors[i])
        if result and distance < max_distance:
            detect = True
            velocities[i] = 1 - ((distance - 0.2) / (max_distance - 0.2))
            if distance < 0.4:
                velocities[i] = 2

    return detect, velocities


def path_follower(sim, xc, yc, tiempo, robot):
    xd = spi.splev(tiempo, xc, der=0)
    yd = spi.splev(tiempo, yc, der=0)

    carpos = sim.getObjectPosition(robot, -1)
    carrot = sim.getObjectOrientation(robot, -1)
    errp = m.sqrt((xd - carpos[0]) ** 2 + (yd - carpos[1]) ** 2)
    angd = m.atan2(yd - carpos[1], xd - carpos[0])
    errh = angdiff(carrot[2], angd)

    v = kv * errp
    omega = kh * errh

    return v2u(v, omega, r, l)


def points_on_circumference(center, r, n):
    return [
        (
            center[0] + (m.cos(2 * m.pi / n * x) * r),  # x
            center[1] + (m.sin(2 * m.pi / n * x) * r),  # y
        )
        for x in range(0, n + 1)
    ]


print("Program started")
simulation_time = 60 * 1

# ------------------------ GENERAR SPLINE ---------------------------------
xarr = np.array([0, 1.0, 1.0, -1.0, -1.0])
yarr = np.array([0, 1.0, -1.0, -1.0, 1.0])
tarr = np.linspace(0, simulation_time, xarr.shape[0])

tnew = np.linspace(0, simulation_time, 60)
xc = spi.splrep(tarr, xarr, s=0)
yc = spi.splrep(tarr, yarr, s=0)
xd_l = spi.splev(tnew, xc, der=0)
yd_l = spi.splev(tnew, yc, der=0)

# ------------------------ Inicializar objetos de CoppeliaSim ---------------------------------
client = RemoteAPIClient(host=get_host())
sim = client.getObject("sim")

n_sensors = 16
robot = sim.getObject("/PioneerP3DX")
motorL = sim.getObject("/PioneerP3DX/leftMotor")
motorR = sim.getObject("/PioneerP3DX/rightMotor")
sensors = []
Rs = []
Vs = []
for i in range(0, 16):
    s = sim.getObject("/PioneerP3DX/ultrasonicSensor[" + str(i) + "]")
    sensors.append(s)
    q = sim.getObjectQuaternion(s, robot)
    Rs.append(q2R(q[0], q[1], q[2], q[3]))
    Vs.append(np.reshape(sim.getObjectPosition(s, robot), (3, 1)))

obstacles = [sim.getObject(f"/Cylinder[{i}]") for i in range(11)]
carpos = sim.getObjectPosition(robot, -1)
carrot = sim.getObjectOrientation(robot, -1)

# ------------------------ GENERAR GRAFICA ---------------------------------
_, ax = plt.subplots()

world_x = [-7.5, -7.5, 7.5, 7.5, -7.5]
world_y = [-7.5, 7.5, 7.5, -7.5, -7.5]
plt.figure(1)
plt.plot(world_x, world_y, c="y", label="mundo")
plt.plot(xd_l, yd_l, label="spline")

for i in obstacles:
    x, y, _ = sim.getObjectPosition(i, -1)
    if i == obstacles[0]:
        plt.plot(x, y, "o", c="g", label="obstaculo")
    else:
        plt.plot(x, y, "o", c="g")

    for x, y in points_on_circumference((x, y), 0.25, 50):
        # plt.plot(x, y, '.', c='g')
        plt.scatter(x, y, s=1, c="g")

plt.plot(xarr, yarr, ".", label="puntos")
plt.plot(xarr[0], yarr[0], "X", c="m", label="inicio")
plt.plot(xarr[-1], yarr[-1], "X", c="r", label="fin")
plt.xlim(-10, 10)
plt.ylim(-10, 10)
plt.title("Trayectoria")
pos = ax.get_position()
ax.set_position([pos.x0, pos.y0, pos.width * 0.85, pos.height])
ax.legend(loc="center right", bbox_to_anchor=(1.32, 0.6))
# plt.legend(loc='center right', bbox_to_anchor=(1.3, 0.5))
plt.show()

# ---------------------------------------------------------------------------
kv = 0.5
kh = 0.8
r = 0.5 * 0.195
l = 2 * 0.1655
errp = 10
max_size = 150
center_x = 150 // 2
center_y = 150 // 2
s = 0.1
initialFloor = 5
cellC = int(initialFloor/s)
cellR = int(initialFloor/s)

x, y, _ = sim.getObjectPosition(robot, -1)
coordinates_x = [x]
coordinates_y = [y]

# Braitenberg
lgains = [-1.0, -0.8, -1.0, -1.2, -2.2, -2.0, -1.8, -2.0]
rgains = [-2.0, -1.8, -2.0, -2.2, -1.2, -1.0, -0.8, -1.0]

#Leer archivo ?
#Crear mapa
print('Creating new map')
occgrid = 0.5*np.ones((cellR, cellR))
tocc = np.zeros((cellR, cellR))
#Calcular centro
center = [int(cellR/2), int(cellC/2)]
t = time.time()
initt = t
niter = 0

# --------------------------- SIMULACIÓN ----------------------------------
sim.startSimulation()

iteraciones = 0
tiempo_requerido = 60
tiempo_total = 60
actual_center = center
expandio = True
terminar = False
iter_no_expandio = 0
while True:
    if terminar:
        break;
    tiempo_inicial_iter = sim.getSimulationTime()
    while sim.getSimulationTime() < tiempo_total:
        tiempo = sim.getSimulationTime() - tiempo_inicial_iter
        carpos = sim.getObjectPosition(robot, -1)

        xw = carpos[0]
        yw = carpos[1]
        xr = center[1] + m.ceil(xw / s)
        yr = center[0] - m.floor(yw / s)
        if xr >= cellC:
            xr = cellC
        if yr >= cellR:
            yr = cellR
        occgrid[yr - 1, xr - 1] = 0

        carrot = sim.getObjectQuaternion(robot, -1)

        uread = []
        ustate = []
        upt = []
        etime = []
        for i in range(0, 16):

            state, distance, point, detectedObj, _ = sim.readProximitySensor(sensors[i])

            uread.append(distance)
            upt.append(point)
            ustate.append(state)

            # Transform detection from sensor frame to robot frame
            if state:
                opos = np.array(point).reshape((3, 1))
            else:
                opos = np.array([0, 0, 1]).reshape((3, 1))

            robs = np.matmul(Rs[i], opos) + Vs[i]

            # Transform detection from robot frame to global frame
            R = q2R(carrot[0], carrot[1], carrot[2], carrot[3])
            rpos = np.array(carpos).reshape((3, 1))
            pobs = np.matmul(R, robs) + rpos

            #Mapa Dinamico
            occgrid, tocc, center = calculateMap(center, s, occgrid, tocc, pobs)
            cellR, cellC = occgrid.shape

            # Transform detection from global frame to occupancy grid cells
            xs = pobs[0]
            ys = pobs[1]
            xo = center[1] + m.ceil(xs / s)
            yo = center[0] - m.floor(ys / s)
            if xo >= cellC:
                xo = cellC
            if yo >= cellR:
                yo = cellR
            if state:
                tocc[yo - 1, xo - 1] = 1
            occgrid = cv2.line(occgrid, (xr - 1, yr - 1), (xo - 1, yo - 1), (0, 0, 0), 1)

        # Reactive navigation block
        ur, ul = path_follower(sim, xc, yc, tiempo, robot)
        for k in range(len(upt) // 2):
            if ustate[k]:
                ul = ul + lgains[k] * (1.0 - uread[k])
                ur = ur + rgains[k] * (1.0 - uread[k])

        sim.setJointTargetVelocity(motorL, ul)
        sim.setJointTargetVelocity(motorR, ur)
        niter = niter + 1
        x, y, _ = sim.getObjectPosition(robot, -1)
        coordinates_x.append(x)
        coordinates_y.append(y)

    if actual_center == center:
        if not expandio and iteraciones - iter_no_expandio > 2:
            terminar = True
        expandio = False
    else:
        iter_no_expandio = iteraciones
        expandio = True

    iteraciones = iteraciones + 1
    tiempo_requerido = tiempo_requerido + 60
    tiempo_total = tiempo_total + tiempo_requerido
    xarr = np.array([carpos[0], -1.0-iteraciones,  1.0+iteraciones, 1.0+iteraciones, -1.0-iteraciones, carpos[0] - iteraciones])
    yarr = np.array([carpos[1], 1.0+iteraciones, 1.0+iteraciones, -1.0-iteraciones,  -1.0-iteraciones, carpos[1] + iteraciones])
    tarr = np.linspace(0, tiempo_requerido, xarr.shape[0])

    tnew = np.linspace(0, tiempo_requerido, 60)
    xc = spi.splrep(tarr, xarr, s=0)
    yc = spi.splrep(tarr, yarr, s=0)
    xd_l = spi.splev(tnew, xc, der=0)
    yd_l = spi.splev(tnew, yc, der=0)

sim.stopSimulation()

# ------------------------------Trayectoria obtenida --------------------------
_, ax = plt.subplots(1, 3)
# ax.plot(coordinates_x, coordinates_y, ".", ls="--", c="b")
ax[1].plot(world_x, world_y, c="y", label="mundo")
ax[1].plot(coordinates_x, coordinates_y, c="b", label="ruta")

for i in obstacles:
    x, y, _ = sim.getObjectPosition(i, -1)
    if i == obstacles[0]:
        ax[1].plot(x, y, "o", c="g", label="obstaculo")
    else:
        ax[1].plot(x, y, "o", c="g")
    for x, y in points_on_circumference((x, y), 0.25, 50):
        # ax.plot(x, y, ".", c="g")
        ax[1].scatter(x, y, s=1, c="g")

ax[1].plot(coordinates_x[0], coordinates_y[0], "X", c="m", label="inicio")
ax[1].plot(coordinates_x[-1], coordinates_y[-1], "X", c="r", label="fin")
ax[1].set_xlim(-10, 10)
ax[1].set_ylim(-10, 10)
ax[1].set_title("Trayectoria Final")
# ax[1].legend(loc='center right', bbox_to_anchor=(1.13, 0.5))
pos = ax[1].get_position()
ax[1].set_position([pos.x0, pos.y0, pos.width * 0.85, pos.height])
ax[1].legend(loc="center right", bbox_to_anchor=(1.32, 0.6))
ax[0].plot(world_x, world_y, c="y")
ax[0].plot(xd_l, yd_l)

for i in obstacles:
    x, y, _ = sim.getObjectPosition(i, -1)
    ax[0].plot(x, y, "o", c="g")
    for x, y in points_on_circumference((x, y), 0.25, 50):
        # plt.plot(x, y, '.', c='g')
        ax[0].scatter(x, y, s=1, c="g")

ax[0].plot(xarr, yarr, ".")
ax[0].plot(xarr[0], yarr[0], "X", c="m")
ax[0].plot(xarr[-1], yarr[-1], "X", c="r")
ax[0].set_xlim(-10, 10)
ax[0].set_ylim(-10, 10)
ax[0].set_title("Trayectoria Original")
ax[2].imshow(tocc + occgrid)
plt.show()