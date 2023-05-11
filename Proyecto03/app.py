import math as m
import os
import platform
import random
import time
import astarmod

import cv2
import matplotlib.pyplot as plt
import numpy as np
import scipy.interpolate as spi
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

def mapToMatrix(point, center, scala):
    return [int(center[0] - point[1]/scala), int(center[1] + point[0]/scala)]
    
def matrixToMap(point, center, scala):
    return [(point[1] - center[1])*scala, (center[0] - point[0])*scala]

if os.path.exists('map.txt'):
    print('Map found. Loading...')
    occgrid = np.loadtxt('map.txt')
    tocc = 1.0*(occgrid > 0.5)
    occgrid[occgrid > 0.5] = 0
    width = 150
    height = 150
    # CENTROS PARA LA TRANSFORMACIÓN
    center_x = 75
    center_y = 75
else:
    print('Creating new map')
    occgrid = 0.5*np.ones((50,50))
    tocc = np.zeros((50,50))
    height = 50
    width = 50
    # CENTROS PARA LA TRANSFORMACIÓN
    center_x = 25
    center_y = 25

mapa = np.uint8(tocc > 0.5)
disk = disk(3)
nmap = binary_dilation(mapa, disk)
map = nmap + occgrid

plt.imshow(map)
plt.show()

cfree = False
while not cfree:
    loc = np.random.randint(0, 150, (4,))
    vals = map[loc[0], loc[1]]
    vale = map[loc[2], loc[3]]
    if vals == 0 and vale == 0:
        cfree = True
print(loc)

route = astarmod.astar(map, (loc[0], loc[1]), (loc[2], loc[3]), allow_diagonal_movement=True)
rr, cc = astarmod.path2cells(route)
map[rr, cc] = 128

plt.imshow(map)
plt.show()