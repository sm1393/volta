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
        self.pub_status = rospy.Publisher('status', String, queue_size=10)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        #Read parameters
        self.map_x = 0.0
        self.map_y = 0.0
        rospy.spin()

    def ui_callback(self,data):
        msg = data.data
        msg = json.loads(msg)
        self.autonomy_flag = msg['autonomy']
        if(msg['stop'] == True):
            self.client.cancel_all_goals()
            self.goal_status = True
        elif(msg['autonomy'] == True and self.goal_status == True):
            latlong = msg['latlong']
            latlong = latlong.split(',')
            self.map_x = float(latlong[0])
            self.map_y = float(latlong[1])
            print("self.map_x:",self.map_x)
            print("self.map_y:",self.map_y)
            self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose" + str(self.map_x) + str(self.map_y) + "is being processed")

    # def feedback_cb(self, feedback):
    #     # rospy.loginfo("Feedback:", + str(feedback))
    #     rospy.loginfo("Feedback for the goal pose" + str(self.map_x) + str(self.map_y) + "received")
    

    def done_cb(self,status,result):
        if status == 1:
            rospy.loginfo('Goal is being processed by the action server')
            data = json.dumps({'id':4})
            self.pub_status(data)

        if status == 2:
            rospy.loginfo("Goal pose received a cancel request after it started executing, completed execution")
            data = json.dumps({'id':2}) #Publish status back to the webapp
            self.pub_status.publish(data)
            return 
            
        elif status == 3:
            rospy.loginfo("Goal pose reached")
            data = json.dumps({'id':3})
            self.pub_status.publish(data)
            self.goal_status = True
            return

        elif status == 4:
            # rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.loginfo("Goal pose was aborted by the Action Server")
            data = json.dumps({'id':1})
            self.pub_status.publish(data)
            self.goal_status = True
            return

        elif status == 5:
            # rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.loginfo("Goal pose has been rejected by the Action server")
            # rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        elif status == 8:
            # rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")
            rospy.loginfo("Goal pose received a cancel request beore it started executing, successfully cancelled")
    
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

        self.client.send_goal(goal,self.done_cb,self.active_cb)
        # wait = client.wait_for_result()
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        # else:
        #     rospy.loginfo("Goal Reached")
        #     self.goal_status = True
            # return client.get_result()


if __name__ == '__main__':
    try:
        Autonomy_obj = Autonomy()
    except:
        rospy.loginfo("Failed to reach goal position")