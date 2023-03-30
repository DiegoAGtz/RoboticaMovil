# -*- coding: utf-8 -*-
"""
@author: DiegoAGtz

@description: Primer proyecto de la materia Robótica Móvil
"""
import math as m
import matplotlib.pyplot as plt
import numpy as np
import platform
import scipy.interpolate as spi
import random
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


def readSensors(sim, sensors):
    max_distance = 0.8
    detect = False
    left_activated = False
    right_activated = False
    detected = [0] * len(sensors)
    activated_list = [0] * len(sensors)
    velocities = [0] * len(sensors)
    for i in range(len(sensors)):
        result, distance, _, _, _ = sim.readProximitySensor(sensors[i])
        if result and distance < max_distance:
            detect = True
            if i < len(sensors) / 2:
                left_activated = True
            else:
                right_activated = True
            detected[i] = distance
            activated_list[i] = 1
            velocities[i] = 1 - ((distance - 0.2) / (max_distance - 0.2))
            # print(f"------------------Velocities{i}: {velocities[i]}------")
            if distance < 0.2:
                # sim.addLog(sim.verbosity_scriptinfos, f"distance from if: {distance}")
                velocities[i] = 2

    return detect, detected, left_activated, right_activated, activated_list, velocities


def path_follower(sim, xc, yc, tiempo, robot):
    tiempo = sim.getSimulationTime()
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
    return [(center[0] + (m.cos(2 * m.pi / n * x) * r),  # x
             center[1] + (m.sin(2 * m.pi / n * x) * r)  # y
            ) for x in range(0, n + 1)]

print("Program started")
simulation_time = 60 * 6

# ------------------------ GENERAR SPLINE ---------------------------------
xarr = np.append([0], [random.randint(-6, 6) for _ in range(9)])
yarr = np.append([0], [random.randint(-6, 6) for _ in range(9)])
print(xarr)
print(yarr)
tarr = np.linspace(0, simulation_time, xarr.shape[0])

tnew = np.linspace(0, simulation_time, 60)
xc = spi.splrep(tarr, xarr, s=0)
yc = spi.splrep(tarr, yarr, s=0)
xd_l = spi.splev(tnew, xc, der=0)
yd_l = spi.splev(tnew, yc, der=0)

# ------------------------ Inicializar objetos de CoppeliaSim ---------------------------------
client = RemoteAPIClient(host=get_host())
sim = client.getObject("sim")

n_sensors = 8
robot = sim.getObject("/PioneerP3DX")
motorL = sim.getObject("/PioneerP3DX/leftMotor")
motorR = sim.getObject("/PioneerP3DX/rightMotor")
sensors = [
    sim.getObject(f"/PioneerP3DX/ultrasonicSensor[{i}]") for i in range(n_sensors)
]
obstacles = [sim.getObject(f"/Cylinder[{i}]") for i in range(11)]
# obstacles = []

# ------------------------ GENERAR GRAFICA ---------------------------------
_, ax = plt.subplots()

world_x = [-7.5, -7.5, 7.5, 7.5, -7.5]
world_y = [-7.5, 7.5, 7.5, -7.5, -7.5]
plt.figure(1)
plt.plot(world_x, world_y, c='y', label='mundo')
plt.plot(xd_l, yd_l, label='spline')

for i in obstacles:
    x, y, _ = sim.getObjectPosition(i, -1)
    if(i == obstacles[0]):
        plt.plot(x, y, "o", c="g", label='obstaculo')
    else:
        plt.plot(x, y, "o", c="g")

    for x, y in points_on_circumference((x,y), 0.25, 50):
        #plt.plot(x, y, '.', c='g')
        plt.scatter(x, y, s=1, c='g')
        
plt.plot(xarr, yarr, ".", label='puntos')
plt.plot(xarr[0], yarr[0], 'X', c='m', label='inicio')
plt.plot(xarr[-1], yarr[-1], 'X', c='r', label='fin')
plt.xlim(-10, 10)
plt.ylim(-10, 10)
plt.title("Trayectoria")
pos = ax.get_position()
ax.set_position([pos.x0, pos.y0, pos.width*0.85, pos.height])
ax.legend(loc='center right', bbox_to_anchor=(1.32, 0.6))
#plt.legend(loc='center right', bbox_to_anchor=(1.3, 0.5))
plt.show()

# ---------------------------------------------------------------------------
kv = 0.3
kh = 0.8
r = 0.5 * 0.195
l = 2 * 0.1655

x, y, _ = sim.getObjectPosition(robot, -1)
coordinates_x = [x]
coordinates_y = [y]

# Braitenberg
braitenbergL = [-0.4, -0.6, -0.8, -1.0, -2.0, -1.8, -1.6, -1.8]
braitenbergR = [-1.8, -1.6, -1.8, -2.0, -1.0, -0.8, -0.6, -0.4]

# --------------------------- SIMULACIÓN ----------------------------------
sim.startSimulation()


while sim.getSimulationTime() < simulation_time:
    tiempo = sim.getSimulationTime()
    ur, ul = path_follower(sim, xc, yc, tiempo, robot)

    object_detected, activated_sensors, l_sensors, r_sensors, _, vel = readSensors(
        sim, sensors
    )

    # Evadir obstaculo
    if object_detected:
        # sim.addLog(sim.verbosity_scriptinfos, f"Objeto detectado: {object_detected}")
        for i in range(8):
            ul = ul + braitenbergL[i] * vel[i]
            ur = ur + braitenbergR[i] * vel[i]

        #if l_sensors and r_sensors:
        #    total_l, total_r = sum(vel[:4]), sum(vel[4:])
        #    if total_l > total_r:
        #        ur = ur - 4
        #    else:
        #        ul = ul - 4    
    
    print(f"Velocidad [{m.floor(tiempo)}]: ul({ul}) - ur({ur})")        

    while (ul > 5.0 or ur > 5.0):
        ul*=0.85
        ur*=0.85
        print(f"Velocidad corregida [{m.floor(tiempo)}]: ul({ul}) - ur({ur})")    

    sim.setJointTargetVelocity(motorL, ul)
    sim.setJointTargetVelocity(motorR, ur)

    x, y, _ = sim.getObjectPosition(robot, -1)
    coordinates_x.append(x)
    coordinates_y.append(y)

sim.stopSimulation()

# ------------------------------Trayectoria obtenida --------------------------
_, ax = plt.subplots(1, 2)
#ax.plot(coordinates_x, coordinates_y, ".", ls="--", c="b")
ax[1].plot(world_x, world_y, c='y', label='mundo')
ax[1].plot(coordinates_x, coordinates_y, c="b", label='ruta')

for i in obstacles:
    x, y, _ = sim.getObjectPosition(i, -1)
    if(i == obstacles[0]):
        ax[1].plot(x, y, "o", c="g", label='obstaculo')
    else:
        ax[1].plot(x, y, "o", c="g")
    for x, y in points_on_circumference((x,y), 0.25, 50):
        #ax.plot(x, y, ".", c="g")
        ax[1].scatter(x, y, s=1, c='g')

ax[1].plot(coordinates_x[0], coordinates_y[0], 'X', c='m', label='inicio')
ax[1].plot(coordinates_x[-1], coordinates_y[-1], 'X', c='r', label='fin')
ax[1].set_xlim(-10, 10)
ax[1].set_ylim(-10, 10)
ax[1].set_title("Trayectoria Final")
#ax[1].legend(loc='center right', bbox_to_anchor=(1.13, 0.5))
pos = ax[1].get_position()
ax[1].set_position([pos.x0, pos.y0, pos.width*0.85, pos.height])
ax[1].legend(loc='center right', bbox_to_anchor=(1.32, 0.6))
ax[0].plot(world_x, world_y, c='y')
ax[0].plot(xd_l, yd_l)

for i in obstacles:
    x, y, _ = sim.getObjectPosition(i, -1)
    ax[0].plot(x, y, "o", c="g")
    for x, y in points_on_circumference((x,y), 0.25, 50):
        #plt.plot(x, y, '.', c='g')
        ax[0].scatter(x, y, s=1, c='g')
        
ax[0].plot(xarr, yarr, ".")
ax[0].plot(xarr[0], yarr[0], 'X', c='m')
ax[0].plot(xarr[-1], yarr[-1], 'X', c='r')
ax[0].set_xlim(-10, 10)
ax[0].set_ylim(-10, 10)
ax[0].set_title("Trayectoria Original")
plt.show()
