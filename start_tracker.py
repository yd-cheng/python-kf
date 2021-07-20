#!/usr/bin/env python2
import sys
sys.path.insert(1, '/home/rdesc/vision-system/python-kf/filters')

from ros_tracker import ROSTracker

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Enter ids of robots separated by spaces')
        print('Exiting')
        sys.exit()

    robot_ids = []

    for i in range(len(sys.argv))[1:]:
        robot_ids.append(sys.argv[i])

    tracker = ROSTracker('kalman_tracker', robot_ids)
    tracker.start()

