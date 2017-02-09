#!/usr/bin/python3.5
from math import sqrt, cos, sin, radians, asin, degrees
import matplotlib
from matplotlib.patches import Circle, Wedge
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt


fig, ax = plt.subplots()
patches = []

l1 = 152
l2 = 112
patches.append(Wedge((0, 143), l1 + l2, -70, 250, width=1))
l3 = sqrt(l1**2 + l2**2 - 2 * l1 * l2 * cos(radians(180 - 120)))
l4 = sqrt(l1**2 + l2**2 - 2 * l1 * l2 * cos(radians(180 - 80)))
a1 = degrees(asin(sin(radians(180-80)) * l2 / l4))
a2 = degrees(asin(sin(radians(180-120)) * l2 / l3))
patches.append(Wedge((0, 143), l4, 217, 250 + a1, width=1))
patches.append(Wedge((0, 143), l3, -70 - a2, 250 - a2, width=1))
patches.append(Wedge((cos(radians(-70))*l1, 143 + sin(radians(-70))*l1), l2, -190, -70, width=1))
patches.append(Wedge((cos(radians(250))*l1, 143 + sin(radians(250))*l1), l2, 250, 330, width=1))
patches.append(Wedge((cos(radians(250))*l1, 143 + sin(radians(250))*l1), l2, 250 - 120, 170, width=1))

p = PatchCollection(patches, alpha=0.4)
ax.add_collection(p)
ax.set_aspect('equal', 'datalim')
ax.set_xlim([-200, 200])
ax.set_ylim([-100, 450])
plt.grid(True)

plt.plot([-600, 600], [0, 0])
plt.savefig("B_postlab_1.png")
plt.show()
