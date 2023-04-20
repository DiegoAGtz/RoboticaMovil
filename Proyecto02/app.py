# -*- coding: utf-8 -*-
"""
@author: DiegoAGtz

@description: Segundo proyecto de la materia Robótica Móvil
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


# ------------------------ Inicializar objetos de CoppeliaSim ---------------------------------
client = RemoteAPIClient(host=get_host())
sim = client.getObject("sim")

n_sensors = 8
robot = sim.getObject("/PioneerP3DX")
motorL = sim.getObject("/PioneerP3DX/leftMotor")
motorR = sim.getObject("/PioneerP3DX/rightMotor")


# --------------------------- SIMULACIÓN ----------------------------------
sim.startSimulation()
sim.stopSimulation()
