#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""Write target here

Write detailed description here

Write typical usage example here

@Modify Time      @Author    @Version    @Description
------------      -------    --------    -----------
11/16/22 10:55 PM   yinzikang      1.0         None
"""

import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

# data = numpy.genfromtxt("result/line/qpos.csv", delimiter=',')
data = numpy.genfromtxt("result/circle/qpos.csv", delimiter=',')
# data = numpy.genfromtxt("result/point/qpos.csv", delimiter=',')

plt.figure(1)
plt.plot(data[:, 0], data[:, 1:7] / numpy.pi * 180)
plt.title('qpos curve')
plt.legend(["joint 1", "joint 2", "joint 3", "joint 4", "joint 5", "joint 6"])
plt.grid()

plt.figure(2)
plt.plot(data[:, 0], data[:, 7:10])
plt.title('xpos curve')
plt.legend(["joint 1", "joint 2", "joint 3"])
plt.grid()

plt.figure(3)
plt.plot(data[:, 0], data[:, 10:14])
plt.title('xmat curve')
plt.legend(["x", "y", "z", "w"])
plt.grid()

fig = plt.figure(4)
ax = fig.gca(projection='3d')
ax.plot(data[:, 7], data[:, 8], data[:, 9], label='xpos')
ax.set_xlabel('X')
ax.set_xlim(0, 1)
ax.set_ylabel('Y')
ax.set_ylim(0, 1)
ax.set_zlabel('Z')
ax.set_zlim(0, 1)
ax.legend()

plt.figure(5)
plt.plot(data[:, 7], data[:, 9],label='x-z')
ax.set_xlabel('X')
ax.set_ylabel('Z')
plt.axis('equal')
plt.grid()

plt.show()
