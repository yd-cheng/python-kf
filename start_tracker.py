#!/usr/bin/env python3
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
        print(sys.argv[i])
        robot_ids.append(sys.argv[i])

    tracker = ROSTracker('filter_{}'.format(robot_ids[0]), robot_ids[0])
    tracker.start()

