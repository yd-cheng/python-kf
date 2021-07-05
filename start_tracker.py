#!/usr/bin/env python3
import sys
sys.path.insert(1, '/home/rdesc/vision-system/python-kf/filters')

from ros_tracker import ROSTracker

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Enter id of robot to track')
        print('Exiting')
        sys.exit()

    tracker = ROSTracker('filter', sys.argv[1])
    tracker.start()

