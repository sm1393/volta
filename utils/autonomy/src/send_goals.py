#!/usr/bin/env python
import numpy as np
import rospy
import json
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Autonomy():
    def __init__(self):
        rospy.init_node('autonomy')
        self.autonomy_flag = False
        #Get the starting position
        #Subscribe to /coord
        rospy.Subscriber('coord',String, self.ui_callback)
        #Read parameters
        pos_a = rospy.get_param('/goal_positions/a')
        pos_b = rospy.get_param('/goal_positions/b')
        pos_c = rospy.get_param('/goal_positions/c')
        self.goal_positions = [pos_a,pos_b,pos_c]
        rospy.spin()

    def ui_callback(self,data):
        msg = data.data
        msg = json.loads(msg)
        self.autonomy_flag = msg['autonomy']
        if(msg['autonomy'] == True):
            self.movebase_client()

    def movebase_client(self):
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        if(self.autonomy_flag == True):
            for i in range(0,len(self.goal_positions)):
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = self.goal_positions[i][0]
                goal.target_pose.pose.position.y = self.goal_positions[i][1]
                goal.target_pose.pose.orientation.w = 1.0

                client.send_goal(goal)
                wait = client.wait_for_result()
                if not wait:
                    rospy.logerr("Action server not available!")
                    rospy.signal_shutdown("Action server not available!")
                else:
                    rospy.loginfo("Goal Reached")
                    # return client.get_result()


if __name__ == '__main__':
    try:
        Autonomy_obj = Autonomy()
    except:
        rospy.loginfo("Failed to reach goal position")