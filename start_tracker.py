#!/usr/bin/env python3
import sys
sys.path.insert(1, '/home/rick/catkin_ws/src/linear-kf/scripts/filters')

from ros_tracker import ROSTracker

if __name__ == '__main__':
    tracker = ROSTracker('filter', 'pose')
    tracker.start()

