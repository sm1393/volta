#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from volta_navigation.srv import Goal

def handle_request(req):
    print( req.x, req.y)

    return 0

if __name__ == "__main__":

    rospy.init_node("test_server")
    rospy.Service('goal_server', Goal, handle_request)
    rospy.spin()

