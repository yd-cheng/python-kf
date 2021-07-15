#!/usr/bin/env python2

import sys
import rosbag
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print("Enter name of rosbag file to plot")
    sys.exit(1)

bag = rosbag.Bag(sys.argv[1])

x_coords = []
y_coords = []

for topic, msg, t in bag.read_messages(topics=[]):
    x_coords.append(msg.pose.position.x)
    y_coords.append(msg.pose.position.y)

bag.close()

fig, ax = plt.subplots()

ax.set_yticks([-1200, -600, 0, 600, 1200], minor=False)
ax.set_xticks([-1200, -600, 0, 600, 1200], minor=False)

ax.yaxis.grid(True, which='major')
ax.xaxis.grid(True, which='major')

ax.invert_yaxis()
ax.yaxis.tick_right()

#ax.plot(1200, 1200, 'or')
#ax.plot(-1200, -1200, 'ob')

plt.scatter(x_coords, y_coords, s=6)
plt.show()


