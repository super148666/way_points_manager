#!/usr/bin/env python

import rospy
from way_points_manager.way_points_manager import WayPointsManager

if __name__ == '__main__':
    wpm = WayPointsManager()
    wpm.spin()
