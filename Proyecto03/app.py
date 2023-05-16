# -*- coding: utf-8 -*-
import math as m
import platform

import astarmod
import matplotlib.pyplot as plt
import numpy as np
import scipy.interpolate as spi
from scipy.ndimage import rotate
from skimage.morphology import binary_dilation, disk
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

def get_foor_size(sim, floor):
    floorChilds = [sim.getObjectChild(floor, i)  for i in range(50) if sim.getObjectChild(floor, i) != -1]
    floorChildsDimensions = [sim.getShapeGeomInfo(i) for i in floorChilds]
    floorChildsPositions = [sim.getObjectPosition(i, -1) for i in floorChilds]
    maxX = max([i[0] for i in floorChildsPositions])
    maxY = max([i[1] for i in floorChildsPositions])
    floorObjectsX = [n for n, i in enumerate(floorChildsPositions) if i[0] == maxX]
    floorObjectsY = [n for n, i in enumerate(floorChildsPositions) if i[1] == maxY]
    floorSize = [sum([floorChildsDimensions[i][2][0] for i in floorObjectsX]), sum([floorChildsDimensions[i][2][1] for i in floorObjectsY])]
    print(f'Tamaño del mapa: {floorSize[0]} x {floorSize[1]} m \n')
    return floorSize

def get_map(sim, floor_size, s, script_path):
    with open(script_path, 'r') as f:
        file_content = f.read() 

    sim.startSimulation()
    obj = sim.createPureShape(0, 0, [0.1 , 0.1, 0.1], 1)    # Object creation
    sm = sim.addScript(1)                                   # Code creation
    sim.associateScriptWithObject(sm, obj)                  # Code association
    sim.setScriptText(sm, file_content)                     # Uploading code
    mapa = sim.callScriptFunction("getMap", sm, [floor_size[1], floor_size[0]], s, obj) # Calling getMap function
    sim.removeObject(obj)                                   # Removing object
    sim.stopSimulation()

    mapa.append(0)
    mapa = np.array(mapa).reshape((int(floor_size[1]/s) + 1, int(floor_size[0]/s) + 1))
    return mapa

def map_dilation(f_map):
    n_disk = disk(3)
    f_map = binary_dilation(f_map, n_disk)
    return f_map

def get_astar_path(d_map, c_init, c_dst):
    route = astarmod.astar(d_map, c_init, c_dst, allow_diagonal_movement=True)
    rr, cc = astarmod.path2cells(route)
    d_map[rr, cc] = 128
    return route, d_map

def map2matrix(points, center, scala):
    p = []
    for i in points:
        p.append([int(center[0] - i[1]/scala), int(center[1] + i[0]/scala)])
    return p
    
def matrix2map(points, center, scala):
    p = []
    for i in points:
        p.append([(i[1] - center[1])*scala, (center[0] - i[0])*scala])
    return p

def v2u(v, omega, r, L):
    ur = v / r + L * omega / (2 * r)
    ul = v / r - L * omega / (2 * r)
    return ur, ul

def angdiff(t1, t2):
    angmag = m.acos(m.cos(t1) * m.cos(t2) + m.sin(t1) * m.sin(t2))
    angdir = m.cos(t1) * m.sin(t2) - m.sin(t1) * m.cos(t2)
    return m.copysign(angmag, angdir)

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

# ------------------------ Inicializar objetos de CoppeliaSim ---------------------------------
client = RemoteAPIClient(host=get_host())
sim = client.getObject("sim")
robot = sim.getObject("/PioneerP3DX")
motorL = sim.getObject("/PioneerP3DX/leftMotor")
motorR = sim.getObject("/PioneerP3DX/rightMotor")
floor = sim.getObject('./*Floor*')
floor_size = get_foor_size(sim, floor)

x, y, _ = sim.getObjectPosition(robot, -1)
s = 0.1
center_map = (floor_size[0]/2, floor_size[1]/2)
center_matrix = (floor_size[0]/(2*s), floor_size[1]/(2*s))

# ------------------------ Analizar mapa ---------------------------------
f_map = get_map(sim, floor_size, s, 'script.lua')
f_map = rotate(f_map, angle = 90)
d_map = map_dilation(f_map)

c_init = map2matrix([(x, y)], center_matrix, s)[0]
c_dst = (0, 0)
c_free = False
while not c_free:
    c_dst = np.random.randint(0, int(floor_size[0]/s), (2,))
    val_e = d_map[c_dst[0], c_dst[1]]
    if not val_e:
        c_free = True
c_dst = tuple(c_dst)

path, p_map = get_astar_path(d_map, c_init, c_dst)
plt.imshow(d_map)
plt.show()

# Creating spline
simulation_time = 60 * 1
map_route = matrix2map(path, center_matrix, 0.1)
# ------------------------ GENERAR SPLINE ---------------------------------
carpos = sim.getObjectPosition(robot, -1)
xarr = np.array([i[0] for i in map_route])
yarr = np.array([i[1] for i in map_route])
tarr = np.linspace(0, simulation_time, xarr.shape[0])

xc = spi.splrep(tarr, xarr, s=0)
yc = spi.splrep(tarr, yarr, s=0)

# ---------------------------------------------------------------------------
kv = 0.5
kh = 0.8
r = 0.5 * 0.195
l = 2 * 0.1655
errp = 10

p_init = matrix2map([c_init], center_matrix, 0.1)
coordinates_x = [p_init[0][0]]
coordinates_y = [p_init[0][1]]

sim.setObjectPosition(robot, -1, [p_init[0][0], p_init[0][1], 0.13879])
print("start ")
sim.startSimulation()
while sim.getSimulationTime() < simulation_time:
    tiempo = sim.getSimulationTime()
    ur, ul = path_follower(sim, xc, yc, tiempo, robot)
    sim.setJointTargetVelocity(motorL, ul)
    sim.setJointTargetVelocity(motorR, ur)

    x, y, _ = sim.getObjectPosition(robot, -1)
    coordinates_x.append(x)
    coordinates_y.append(y)

sim.stopSimulation()

# ------------------------------Trayectoria obtenida --------------------------
_, ax = plt.subplots(1, 2)
ax[0].imshow(d_map)

ax[1].plot(xarr, yarr, ".", label="puntos")
ax[1].plot(xarr[0], yarr[0], "X", c="m", label="inicio")
ax[1].plot(xarr[-1], yarr[-1], "X", c="r", label="fin")
ax[1].set_xlim(-center_map[0]/2 - 3, center_map[0]/2 + 3)
ax[1].set_ylim(-center_map[1]/2 - 3, center_map[1]/2 + 3)
ax[1].set_title("Trayectoria obtenida")
plt.show()