import math as m
import os
import platform
import random
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
import scipy.interpolate as spi
# from zmqRemoteApi import RemoteAPIClient


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

plt.imshow(tocc + occgrid)
plt.show()