import numpy as np
import time
import math as m
import random
import sys
import platform
import matplotlib.pyplot as plt
import os
from zmqRemoteApi import RemoteAPIClient

# from skimage.draw import line
import cv2


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


def initialMapDynamic():
    cellC = 0
    cellR = 0
    for i in range(0, 16):
        state, distance, point, _, _ = sim.readProximitySensor(usensor[i])
        # Transform detection from sensor frame to robot frame
        if state == True:
            opos = np.array(point).reshape((3, 1))
        else:
            opos = np.array([0, 0, 1]).reshape((3, 1))

        robs = np.matmul(Rs[i], opos) + Vs[i]
        # Transform detection from robot frame to global frame
        R = q2R(carrot[0], carrot[1], carrot[2], carrot[3])
        rpos = np.array(carpos).reshape((3, 1))
        pobs = np.matmul(R, robs) + rpos
        # Global Frame -> Initial Frame
        pobs[0] = abs(pobs[0] - centerWorld[0])
        pobs[1] = abs(pobs[1] - centerWorld[1])
        if pobs[0] > cellC:
            cellC = pobs[0]
        if pobs[1] > cellR:
            cellR = pobs[1]

    # Crear mapa
    cellR = int((cellR * 2) / s)
    cellC = int((cellC * 2) / s)
    print("Creating new map")
    occgrid = 0.5 * np.ones((cellR, cellC))
    tocc = np.zeros((cellR, cellC))
    # Calcular centro
    center = [int(cellR / 2), int(cellC / 2)]

    return [occgrid, tocc, cellR, cellC, center]


def initialMapSuperDynamic():
    # sizeMap = {(13,14,15,0,1,2):0, (3,4):0, (5,6,7,8,9,10):0, (11,12):0}
    sizeMap = {(1, 2, 3, 4, 5, 6): 0, (7, 8): 0, (9, 10, 11, 12, 13, 14): 0, (15, 0): 0}
    cellC = 0
    cellR = 0

    for k in list(sizeMap.keys()):

        for i in k:
            state, distance, point, _, _ = sim.readProximitySensor(usensor[i])
            # Transform detection from sensor frame to robot frame
            if state == True:
                opos = np.array(point).reshape((3, 1))
            else:
                opos = np.array([0, 0, 1]).reshape((3, 1))

            robs = np.matmul(Rs[i], opos) + Vs[i]
            # Transform detection from robot frame to global frame
            R = q2R(carrot[0], carrot[1], carrot[2], carrot[3])
            rpos = np.array(carpos).reshape((3, 1))
            pobs = np.matmul(R, robs) + rpos
            # Global Frame -> Initial Frame
            pobs[0] = abs(pobs[0] - centerWorld[0])
            pobs[1] = abs(pobs[1] - centerWorld[1])
            if len(k) == 6:
                if pobs[0] > sizeMap[k]:
                    sizeMap[k] = pobs[0]
            else:
                if pobs[1] > sizeMap[k]:
                    sizeMap[k] = pobs[1]
        if len(k) == 2:
            cellR += sizeMap[k]
        else:
            cellC += sizeMap[k]

    print(cellR)
    print(cellC)
    # Crear mapa
    cellR = int(cellR / s)
    cellC = int(cellC / s)
    # cellR = int(sizeMap[(7,8)]/s) + int(sizeMap[(15,0)]/s)
    # cellC = int(sizeMap[(1,2,3,4,5,6)]/s) + int(sizeMap[(9,10,11,12,13,14)]/s)
    print(cellR)
    print(cellC)
    print("Creating new map")
    occgrid = 0.5 * np.ones((cellR, cellC))
    tocc = np.zeros((cellR, cellC))
    # Calcular centro
    center = [int(sizeMap[(15, 0)] / s), int(sizeMap[(9, 10, 11, 12, 13, 14)] / s)]
    print(center)

    return [occgrid, tocc, cellR, cellC, center]


def calculateMap(center, scala, Map, obstacles, sensor):

    sSensor = [int(i / scala) for i in sensor[:2]]
    cSensor = [center[0] - sSensor[1], center[1] + sSensor[0]]
    # cSensor = [center[0] - sSensor[0], center[1] + sSensor[1]]

    print(cSensor)
    newMap = np.copy(Map)
    newObstacles = np.copy(obstacles)
    newCenter = center
    size = newMap.shape

    combineMap = (
        lambda m1, m2, n: np.hstack((m1, m2)) if bool(n) else np.vstack((m1, m2))
    )

    for i, _ in enumerate(size):
        if cSensor[i] > size[i]:
            if i:
                newSize = (size[i - 1], int(cSensor[i] - size[i]))
            else:
                newSize = (int(cSensor[i] - size[i]), size[i + 1])
            newMap = combineMap(np.copy(newMap), 0.5 * np.ones(newSize), i)
            newObstacles = combineMap(np.copy(newObstacles), np.zeros(newSize), i)
            size = newMap.shape

        if cSensor[i] < 0:
            if i:
                newSize = (size[i - 1], int(abs(cSensor[i])))
            else:
                newSize = (int(abs(cSensor[i])), size[i + 1])
            newMap = combineMap(0.5 * np.ones(newSize), np.copy(newMap), i)
            newObstacles = combineMap(np.zeros(newSize), np.copy(newObstacles), i)
            newCenter[i] += newSize[i]
            size = newMap.shape

    return [newMap, newObstacles, newCenter]


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


client = RemoteAPIClient(host=get_host())
sim = client.getObject("sim")

motorL = sim.getObject("/PioneerP3DX/leftMotor")
motorR = sim.getObject("/PioneerP3DX/rightMotor")
robot = sim.getObject("/PioneerP3DX")
# floor = sim.getObject('/ResizableFloor_5_25')
# floor = sim.getObject('/ResizableFloorMedium')
floor = sim.getObject("./*Floor*")

sim.startSimulation()

# Assigning handles to the ultrasonic sensors
usensor = []
Rs = []
Vs = []
for i in range(0, 16):
    s = sim.getObject("/PioneerP3DX/ultrasonicSensor[" + str(i) + "]")
    usensor.append(s)
    q = sim.getObjectQuaternion(s, robot)
    Rs.append(q2R(q[0], q[1], q[2], q[3]))
    Vs.append(np.reshape(sim.getObjectPosition(s, robot), (3, 1)))

carpos = sim.getObjectPosition(robot, -1)
# carrot = sim.getObjectOrientation(robot, -1)
carrot = sim.getObjectQuaternion(robot, -1)

Kv = 0.5
Kh = 2.5
xd = 3
yd = 3
hd = 0
r = 0.1
L = 0.2
errp = 10

# Leer archivo ?
s = 0.1
cellC = 0
cellR = 0
centerWorld = sim.getObjectPosition(robot, -1)

# MAPA INICIAL MUY DINAMICO
# occgrid, tocc, cellR, cellC, center = initialMapSuperDynamic()
# print(f'Tamaño de celda: {s} m')
# print(f'Cantidad de celdas: {cellR} x {cellC}')
# print(f'Centro: {center} \n')

# Calcular mapa inicial DINAMICO PERO NO TANTO
occgrid, tocc, cellR, cellC, center = initialMapDynamic()

print(f"Tamaño de celda: {s} m")
print(f"Cantidad de celdas: {cellR} x {cellC}")
print(f"Centro: {center} \n")


t = time.time()

initt = t
niter = 0
while time.time() - t < 30:
    carpos = sim.getObjectPosition(robot, -1)

    xw = carpos[0] - centerWorld[0]
    yw = carpos[1] - centerWorld[1]
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

        state, distance, point, _, _ = sim.readProximitySensor(usensor[i])

        uread.append(distance)
        upt.append(point)
        ustate.append(state)

        # Transform detection from sensor frame to robot frame
        if state == True:
            opos = np.array(point).reshape((3, 1))
        else:
            opos = np.array([0, 0, 1]).reshape((3, 1))

        robs = np.matmul(Rs[i], opos) + Vs[i]

        # Transform detection from robot frame to global frame

        R = q2R(carrot[0], carrot[1], carrot[2], carrot[3])
        rpos = np.array(carpos).reshape((3, 1))
        pobs = np.matmul(R, robs) + rpos

        # Global Frame -> Initial Frame
        # sensor = [c - centerWorld[i] for i, c in enumerate(pobs[:2])]
        pobs[0] = pobs[0] - centerWorld[0]
        pobs[1] = pobs[1] - centerWorld[1]

        # Mapa Dinamico
        occgrid, tocc, center = calculateMap(center, s, occgrid, tocc, pobs)
        cellR, cellC = occgrid.shape
        print(f"Celdas: {cellR} x {cellC} \n")
        # print(f'Obstaculos: {tocc.shape[0]} x {tocc.shape[1]} \n')

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
    ul, ur = 2, 2
    lgains = [-1.2, -0.8, -1.0, -1.2, -2.2, -2.0, -1.8, -2.0]
    rgains = [-2.0, -1.8, -2.0, -2.2, -1.2, -1.0, -0.8, -1.2]
    for k in range(len(upt)):
        if ustate[k]:
            ul = ul + lgains[k] * (1.0 - uread[k])
            ur = ur + rgains[k] * (1.0 - uread[k])

    if (
        (uread[2] < 0.3 and uread[2] > 0)
        or (uread[3] < 0.5 and uread[3] > 0)
        or (uread[4] < 0.5 and uread[4] > 0)
        or (uread[5] < 0.3 and uread[5] > 0)
    ):
        t_tmp = time.time()
        while time.time() - t_tmp < 0.5:
            ul, ur = -2, -2
            sim.setJointTargetVelocity(motorL, ul)
            sim.setJointTargetVelocity(motorR, ur)
        degr = random.randint(45, 135) * random.choice([-1, 1])
        sign = random.choice([-1, 1])
        print(f"Grados a dar: {degr} - Segundos: {0.5 * (degr // 30)}")
        t_tmp = time.time()
        while time.time() - t_tmp < 0.5 * (degr // 30):
            ur, ul = v2u(0, sign * 2.0 * m.pi / 12.0, r, L)
            sim.setJointTargetVelocity(motorL, ul)
            sim.setJointTargetVelocity(motorR, ur)
    else:
        sim.setJointTargetVelocity(motorL, ul)
        sim.setJointTargetVelocity(motorR, ur)
        niter = niter + 1
    niter = niter + 1

print(lgains)
print(rgains)
finalt = time.time()
print("Avg time per iteration ", (finalt - initt) / niter)

sim.setJointTargetVelocity(motorL, 0)
sim.setJointTargetVelocity(motorR, 0)

sim.stopSimulation()

finalMap = tocc + occgrid
for r, rc in enumerate(finalMap):
    for c, _ in enumerate(rc):
        if finalMap[r][c] > 1:
            finalMap[r][c] = 1

plt.imshow(finalMap)

plt.show()
np.savetxt("mapDynamic.txt", tocc + occgrid)
