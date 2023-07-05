#!/usr/bin/env python
import numpy as np
import rospy
import json
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


#NOTE: Command to cancel goals: rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}

class Autonomy():
    def __init__(self):
        rospy.init_node('autonomy')
        self.autonomy_flag = False
        self.goal_status = True #Flag to indicate that the vehicle is capable of accepting goals
        #Subscribe to /coord
        rospy.Subscriber('coord',String, self.ui_callback)
        #Read parameters
        self.map_x = 0.0
        self.map_y = 0.0
        rospy.spin()

    def ui_callback(self,data):
        msg = data.data
        msg = json.loads(msg)
        self.autonomy_flag = msg['autonomy']
        if(msg['autonomy'] == True and self.goal_status == True):
            latlong = msg['latlong']
            latlong = latlong.split(',')
            self.map_x = float(latlong[0])
            self.map_y = float(latlong[1])
            print("self.map_x:",self.map_x)
            print("self.map_y:",self.map_y)
            self.movebase_client()

    def movebase_client(self):
        self.goal_status = False
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.map_x
        goal.target_pose.pose.position.y = self.map_y
        goal.target_pose.pose.orientation.w = 1.0

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            rospy.loginfo("Goal Reached")
            self.goal_status = True
            # return client.get_result()


if __name__ == '__main__':
    try:
        Autonomy_obj = Autonomy()
    except:
        rospy.loginfo("Failed to reach goal position")