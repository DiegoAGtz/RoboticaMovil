import matplotlib.pyplot as plt
import numpy as np
import cv2
import astarmod

map = np.zeros((100,100), np.uint8)
Ncircles = 50
xc = np.random.randint(0, 100, (Ncircles,))
yc = np.random.randint(0, 100, (Ncircles,))
print('{} {}'.format(xc[0], yc[0]))

for k in range(Ncircles):
    cv2.circle(map, (xc[k], yc[k]), np.random.randint(1,10), (255,255,255), thickness=-1)

cfree = False
while not cfree:
    loc = np.random.randint(0, 100, (4,))
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