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


def points_on_circumference(center, r, n):
    return [
        (
            center[0] + (m.cos(2 * m.pi / n * x) * r),  # x
            center[1] + (m.sin(2 * m.pi / n * x) * r),  # y
        )
        for x in range(0, n + 1)
    ]


def readSensors(sim, sensors):
    max_distance = 0.8
    detect = False
    left_activated = False
    right_activated = False
    detected = [0] * len(sensors)
    activated_list = [0] * len(sensors)
    velocities = [0] * len(sensors)
    distances = [0] * len(sensors)
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
            distances[i] = distance
            print(f"------------------Velocities{i}: {velocities[i]}------")
            if distance < 0.05:
                sim.addLog(sim.verbosity_scriptinfos, f"distance from if: {distance}")
                velocities[i] = 2

    return (
        detect,
        detected,
        left_activated,
        right_activated,
        activated_list,
        velocities,
        distances,
    )


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

    # return v2u(v, omega, r, l)
    return v, omega


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
obstacles = [sim.getObject(f"/Cylinder[{i}]") for i in range(11)]
# obstacles = [sim.getObject(f"/Cuboid[{i}]") for i in range(4, 12)]
# obstacles = []

plt.figure(1)
plt.plot(xd_l, yd_l)
plt.plot(xarr, yarr, ".")
for i in obstacles:
    x, y, _ = sim.getObjectPosition(i, -1)
    plt.plot(x, y, "X", c="g")
    for x, y in points_on_circumference((x, y), 0.25, 50):
        plt.plot(x, y, ".", c="g")
plt.xlim(-8, 8)
plt.ylim(-8, 8)
# ax.set_aspect('equal')
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
braitenbergL = [-0.4, -0.6, -0.4, -0.2, -2.0, -1.8, -1.6, -1.8]
braitenbergR = [-1.8, -1.6, -1.8, -2.0, -0.4, -0.6, -0.8, -0.6]

# --------------------------- SIMULACIÓN ----------------------------------
sim.startSimulation()


while sim.getSimulationTime() < simulation_time:
    tiempo = sim.getSimulationTime()
    v, omega = path_follower(sim, xc, yc, tiempo, robot)
    if v > 0.4:
        v = 0.4
    if omega > 2.0:
        omega = 2.0
    elif omega < -2.0:
        omega = -2.0
    ur, ul = v2u(v, omega, r, l)

    (
        object_detected,
        activated_sensors,
        l_sensors,
        r_sensors,
        activated_list,
        vel,
        distances,
    ) = readSensors(sim, sensors)

    # Evadir obstaculo
    if object_detected:
        sim.addLog(sim.verbosity_scriptinfos, f"Objeto detectado: {object_detected}")
        for i in range(8):
            ul = ul + braitenbergL[i] * vel[i]
            ur = ur + braitenbergR[i] * vel[i]

        # min_d = 10
        # print(f"------===============Distances: {distances}")
        # for d in distances:
        #     if d < min_d and d != 0:
        #         min_d = d

        # min_index = distances.index(min_d)
        # print(
        #    f"min_d: {min_d} - min_index: {min_index} - distance: {distances[min_index]}"
        # )
        # if (
        #    min_d < 0.3
        #    and activated_list[min_index] == 1
        #    and min_index != 0
        #    and min_index != 7
        # ):
        #    print(f".........min-distance: {min_d}")
        #    total_l, total_r = sum(activated_list[:4]), sum(activated_list[4:])
        #    if total_l > total_r:
        #        ur = -1
        #    else:
        #        ul = -1

        # print(
        #    f"Velocidad al segundo {tiempo}: ul({ul}) - ur({ur}), Detectado: {object_detected}"
        # )

    sim.setJointTargetVelocity(motorL, ul)
    sim.setJointTargetVelocity(motorR, ur)

    x, y, _ = sim.getObjectPosition(robot, -1)
    coordinates_x.append(x)
    coordinates_y.append(y)

sim.stopSimulation()

# ------------------------------Trayectoria obtenida --------------------------
_, ax = plt.subplots(1, 2)
ax[0].plot(coordinates_x, coordinates_y, ls="-", c="b")
ax[0].plot(coordinates_x[0], coordinates_y[0], "X", c="r")
for i in obstacles:
    x, y, _ = sim.getObjectPosition(i, -1)
    ax[0].plot(x, y, "X", c="g")
    for x, y in points_on_circumference((x, y), 0.25, 50):
        ax[0].plot(x, y, ".", c="g")
# ax.set_aspect('equal', 'box')
ax[0].set_xlim(-8, 8)
ax[0].set_ylim(-8, 8)
ax[0].set_title("Trayectoria Obtenida")

ax[1].plot(xd_l, yd_l, c="b")
ax[1].plot(xarr, yarr, ".", c="b")
for i in obstacles:
    x, y, _ = sim.getObjectPosition(i, -1)
    ax[1].plot(x, y, "X", c="g")
    for x, y in points_on_circumference((x, y), 0.25, 50):
        ax[1].plot(x, y, ".", c="g")
ax[1].set_xlim(-8, 8)
ax[1].set_ylim(-8, 8)
ax[1].set_title("Trayectoria Original")
plt.show()
