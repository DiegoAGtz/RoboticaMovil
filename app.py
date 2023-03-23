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


def show_tray(x, y):
    _, ax = plt.subplots()
    ax.plot(x, y, ".", ls="--", c="b")
    ax.plot(x[0], y[0], "X", c="r")
    ax.set_title("Trayectoria")
    plt.show()


def path_tray_err(sim, robot, motor_r, motor_l, xd, yd):
    # for xd, yd in zip(xd_l, yd_l):
    errp = 1000
    while errp > 0.1:
        carpos = sim.getObjectPosition(robot, -1)
        carrot = sim.getObjectOrientation(robot, -1)
        errp = m.sqrt((xd - carpos[0]) ** 2 + (yd - carpos[1]) ** 2)
        angd = m.atan2(yd - carpos[1], xd - carpos[0])
        errh = angdiff(carrot[2], angd)
        print("Distance to goal: {} Heading error: {}".format(errp, errh))

        v = kv * errp
        omega = kh * errh

        ur, ul = v2u(v, omega, r, l)
        sim.setJointTargetVelocity(motor_l, ul)
        sim.setJointTargetVelocity(motor_r, ur)

        x, y, _ = sim.getObjectPosition(robot, -1)
        coordinates_x.append(x)
        coordinates_y.append(y)


"""
@description:
sensors_l -> [0, 1, 2, 3]
sensors_r -> [7, 6, 5, 4]
"""


def evade(sim, motor_r, motor_l, sensors):
    ur = sim.getJointTargetVelocity(motorR)
    ul = sim.getJointTargetVelocity(motorL)
    while True:
        activated_sensor = False
        for i in range(nSensors):
            result, distance, _, _, _ = sim.readProximitySensor(sensors[i])
            if result and distance < noDetectionDist:
                if distance < maxDetectionDist:
                    distance = maxDetectionDist
                    detect[i] = 1 - (
                        (distance - maxDetectionDist)
                        / (noDetectionDist - maxDetectionDist)
                    )
                    activated_sensor = True
                else:
                    detect[i] = 0

        if not activated_sensor:
            break

        for i in range(nSensors):
            print(f"{ur} - {ul}")
            # ur = braitenbergR[i] * detect[i] + ur
            # ul = braitenbergL[i] * detect[i] + ul

            ur = -2
            ul = -2

            sim.setJointTargetVelocity(motor_l, ul)
            sim.setJointTargetVelocity(motor_r, ur)


print("Program started")

points_number = 120

xarr = np.append([0], [random.randint(-7, 7) for _ in range(9)])
yarr = np.append([0], [random.randint(-7, 7) for _ in range(9)])
tarr = np.linspace(0, 10, xarr.shape[0])

tnew = np.linspace(0, 10, points_number)
xc = spi.splrep(tarr, xarr, s=0)
yc = spi.splrep(tarr, yarr, s=0)
xnew = spi.splev(tnew, xc, der=0)
ynew = spi.splev(tnew, yc, der=0)

plt.figure(1)
plt.plot(xnew, ynew)
plt.plot(xarr, yarr, ".")
plt.title("Trayectoria")
plt.show()

client = RemoteAPIClient(host=get_host())
sim = client.getObject("sim")

motorL = sim.getObject("/PioneerP3DX/leftMotor")
motorR = sim.getObject("/PioneerP3DX/rightMotor")
robot = sim.getObject("/PioneerP3DX")
nSensors = 8
sensors = [
    sim.getObject(f"/PioneerP3DX/ultrasonicSensor[{i}]") for i in range(nSensors)
]

xd_l = xnew
yd_l = ynew

kv = 0.3
kh = 0.8
r = 0.5 * 0.195
l = 2 * 0.1655

x, y, _ = sim.getObjectPosition(robot, -1)
coordinates_x = [x]
coordinates_y = [y]

noDetectionDist = 0.5
maxDetectionDist = 0.2
detect = [0.0] * 8
braitenbergL = [-0.2, -0.4, -0.6, -0.8, -1, -1.2, -1.4, -1.6]
braitenbergR = [-1.6, -1.4, -1.2, -1, -0.8, -0.6, -0.4, -0.2]
v0 = 2

# --------------------------- SIMULATION ----------------------------------

sim.startSimulation()

# while sim.getSimulationTime() < points_number - 2:
#     xd = xd_l[m.floor(sim.getSimulationTime())]
#     yd = yd_l[m.floor(sim.getSimulationTime())]
#     print(f"{m.floor(sim.getSimulationTime())} - {yd}")
#     carpos = sim.getObjectPosition(robot, -1)
#     carrot = sim.getObjectOrientation(robot, -1)
#     errp = m.sqrt((xd - carpos[0]) ** 2 + (yd - carpos[1]) ** 2)
#     angd = m.atan2(yd - carpos[1], xd - carpos[0])
#     errh = angdiff(carrot[2], angd)
#     print("Distance to goal: {} Heading error: {}".format(errp, errh))
#
#     v = kv * errp
#     omega = kh * errh
#
#     ur, ul = v2u(v, omega, r, l)
#
#     sim.setJointTargetVelocity(motorL, ul)
#     sim.setJointTargetVelocity(motorR, ur)
#
#     x, y, _ = sim.getObjectPosition(robot, -1)
#     coordinates_x.append(x)
#     coordinates_y.append(y)

for xd, yd in zip(xd_l, yd_l):
    # evade(sim, motorR, motorL, sensors)
    path_tray_err(sim, robot, motorR, motorL, xd, yd)

    if sim.getSimulationTime() > 120:
        break

sim.stopSimulation()
show_tray(coordinates_x, coordinates_y)
