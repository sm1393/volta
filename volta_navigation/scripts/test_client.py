#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from volta_navigation.srv import Goal

if __name__ == "__main__":
    rospy.init_node("send_goal")
    rospy.wait_for_service('goal_server')


    goals = rospy.ServiceProxy('goal_server', Goal)


    x = 0
    y = 0
    resp1 = goals(x, y)

    print(resp1)