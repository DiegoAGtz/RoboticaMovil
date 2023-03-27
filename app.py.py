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
import math

def points_on_circumference(center, r, n):
    return [(center[0] + (m.cos(2 * m.pi / n * x) * r),  # x
             center[1] + (m.sin(2 * m.pi / n * x) * r)  # y
            ) for x in range(0, n + 1)]

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
            print(f"------------------Velocities{i}: {velocities[i]}------")
            if distance < 0.3:
                sim.addLog(sim.verbosity_scriptinfos, f"distance from if: {distance}")
                velocities[i] = 1

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


print("Program started")
simulation_time = 60 * 4

# ------------------------ GENERAR SPLINE ---------------------------------
xarr = np.append([0], [random.randint(-6, 6) for _ in range(9)])
yarr = np.append([0], [random.randint(-6, 6) for _ in range(9)])
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
obstacles = [sim.getObject(f"/Cylinder[{i}]") for i in range(7)]
# obstacles = []
#_, ax = plt.subplots()
plt.figure(1)
plt.plot(xd_l, yd_l)
plt.plot(xarr, yarr, ".")
for i in obstacles:
    x, y, _ = sim.getObjectPosition(i, -1)
    plt.plot(x, y, "X", c="g")
    for x, y in points_on_circumference((x,y), 0.25, 50):
        plt.plot(x, y, ".", c="g")
plt.xlim(-8, 8)
plt.ylim(-8, 8)
#ax.set_aspect('equal')
plt.title("Trayectoria")
plt.show()

kv = 0.3
kh = 0.8
r = 0.5 * 0.195
l = 2 * 0.1655

x, y, _ = sim.getObjectPosition(robot, -1)
coordinates_x = [x]
coordinates_y = [y]

# Braitenberg
braitenbergL = [-0.4, -0.6, -0.8, -1.0, -1.2, -1.4, -1.6, -1.8]
braitenbergR = [-1.8, -1.6, -1.4, -1.2, -1.0, -0.8, -0.6, -0.4]

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
        sim.addLog(sim.verbosity_scriptinfos, f"Objeto detectado: {object_detected}")
        for i in range(8):
            ul = ul + braitenbergL[i] * vel[i]
            ur = ur + braitenbergR[i] * vel[i]

        if l_sensors and r_sensors:
            total_l, total_r = sum(vel[:4]), sum(vel[4:])
            if total_l > total_r:
                ur = ur - 4
            else:
                ul = ul - 4

    print(
        f"Velocidad al segundo {tiempo}: ul({ul}) - ur({ur}), Detectado: {object_detected}"
    )

    sim.setJointTargetVelocity(motorL, ul)
    sim.setJointTargetVelocity(motorR, ur)

    x, y, _ = sim.getObjectPosition(robot, -1)
    coordinates_x.append(x)
    coordinates_y.append(y)

sim.stopSimulation()

# ------------------------------Trayectoria obtenida --------------------------
_, ax = plt.subplots()
ax.plot(coordinates_x, coordinates_y, ".", ls="--", c="b")
ax.plot(coordinates_x[0], coordinates_y[0], "X", c="r")
for i in obstacles:
    x, y, _ = sim.getObjectPosition(i, -1)
    ax.plot(x, y, "X", c="g")
    for x, y in points_on_circumference((x,y), 0.25, 50):
        ax.plot(x, y, ".", c="g")
#ax.set_aspect('equal', 'box')
ax.set_xlim(-8, 8)
ax.set_ylim(-8, 8)
ax.set_title("Trayectoria Final")
plt.show()


