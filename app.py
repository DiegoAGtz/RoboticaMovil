# -*- coding: utf-8 -*-
"""
@author: DiegoAGtz

@description: Escriba un programa en Python, dentro de un 
    solo archivo .py que será su entrega de la actividad. Este 
    programa deberá generar una lista de 10 puntos aleatorios 
    en el plano XY, y debe a continuación calcular una 
    trayectoria suave que pase por todos los puntos generados 
    (puede usar b-splines). Haga que la trayectoria consista 
    de 200 instantes de tiempo, con sus correspondientes 
    coordenadas X y Y.
"""
import numpy as np
import scipy.interpolate as spi
import matplotlib.pyplot as plt

import platform
import math as m
import matplotlib.pyplot as plt
from zmqRemoteApi import RemoteAPIClient
import time

def get_host():
    so = platform.system()
    if so != "Linux":
        return 'localhost'

    ip = ''
    with open('/etc/resolv.conf', 'r') as f:
        data = f.readlines()
        ip = data[-1][:-1]
        ip = ip[11:]
    return ip

def v2u(v, omega, r, L):
    ur = v/r + L*omega/(2*r)
    ul = v/r - L*omega/(2*r)
    return ur, ul

def angdiff(t1, t2):
    """
    Compute the angle difference, t2-t1, restricting the result to the [-pi,pi] range
    """
    # The angle magnitude comes from the dot product of two vectors
    angmag = m.acos(m.cos(t1)*m.cos(t2)+m.sin(t1)*m.sin(t2))
    # The direction of rotation comes from the sign of the cross product of two vectors
    angdir = m.cos(t1)*m.sin(t2)-m.sin(t1)*m.cos(t2)
    return m.copysign(angmag, angdir)


print('Program started')

xarr= np.append([0], np.random.rand(9) * 100 // 20)
yarr = np.append([0], np.random.rand(9) * 100 // 20)
tarr = np.linspace(0, 10, xarr.shape[0])

tnew = np.linspace(0, 10, 60)
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
sim = client.getObject('sim')

motorL=sim.getObject("/PioneerP3DX/leftMotor")
motorR=sim.getObject("/PioneerP3DX/rightMotor")
robot = sim.getObject("/PioneerP3DX")

xd_l = xnew
yd_l = ynew

kv = 0.3
kh = 0.8
r = 0.5*0.195
l = 2*0.1655

x, y, _ = sim.getObjectPosition(robot, -1)
coordinates_x = [x]
coordinates_y = [y]

fig, ax = plt.subplots()

sim.startSimulation()

# for xd, yd in zip(xd_l, yd_l):
#     errp = 1000
#     while errp > 0.1:
#         carpos = sim.getObjectPosition(robot, -1)
#         carrot = sim.getObjectOrientation(robot, -1)
#         errp = m.sqrt((xd-carpos[0])**2 + (yd-carpos[1])**2)
#         angd = m.atan2(yd-carpos[1], xd-carpos[0])
#         errh = angdiff(carrot[2], angd)
#         print('Distance to goal: {} Heading error: {}'.format(errp, errh))
#     
#         v = kv*errp
#         omega = kh*errh
#     
#         ur, ul = v2u(v, omega, r, l)
#         sim.setJointTargetVelocity(motorL, ul)
#         sim.setJointTargetVelocity(motorR, ur)
# 
#     x, y, _ = sim.getObjectPosition(robot, -1)
#     coordinates_x.append(x)
#     coordinates_y.append(y)

while sim.getSimulationTime() < 60:
    xd = xd_l[m.floor(sim.getSimulationTime())]
    yd = yd_l[m.floor(sim.getSimulationTime())]
    print(f'{m.floor(sim.getSimulationTime())} - {yd}')
    carpos = sim.getObjectPosition(robot, -1)
    carrot = sim.getObjectOrientation(robot, -1)
    errp = m.sqrt((xd-carpos[0])**2 + (yd-carpos[1])**2)
    angd = m.atan2(yd-carpos[1], xd-carpos[0])
    errh = angdiff(carrot[2], angd)
    print('Distance to goal: {} Heading error: {}'.format(errp, errh))

    v = kv*errp
    omega = kh*errh

    ur, ul = v2u(v, omega, r, l)
    sim.setJointTargetVelocity(motorL, ul)
    sim.setJointTargetVelocity(motorR, ur)

    x, y, _ = sim.getObjectPosition(robot, -1)
    coordinates_x.append(x)
    coordinates_y.append(y)
    
sim.stopSimulation()

time.sleep(1)

ax.plot(coordinates_x, coordinates_y, ".", ls="--", c="b")
# Muestra una X roja en el punto de partida del robot
ax.plot(coordinates_x[0], coordinates_y[0], "X", c="r")
ax.set_title("Trayectoria")
plt.show()
