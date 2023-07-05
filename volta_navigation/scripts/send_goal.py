#!/usr/bin/python

import rospy
import math
from turtle import distance
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class SendGoal:

    def __init__(self):
        self.robot_position = []
        rospy.init_node("send_goal")
        rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped, self.amcl_pose_callback )
        rospy.Subscriber('/dummy_goal_topid',Float32MultiArray, self.goal_callback )
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        

        self.x_max =  1.3
        self.y_max =  1.10
        self.x_min = -0.8
        self.y_min = -1.10


        self.goal = []
        rospy.spin()

    def movebase_client(self, goal):

        
        print("Waiting for server")
        self.client.wait_for_server()
        print("MoveBase Server Up!")

        x,y = goal

        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose.position.x = x
        move_base_goal.target_pose.pose.position.y = y
        move_base_goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(move_base_goal)

        self.client.wait_for_result()

        return self.client.get_result()


    def isValid(self, goal):
        x,y = goal
        if ( x < self.x_max and x > self.x_min ) and ( y < self.y_max and y > self.y_min ) :
            return True

        return False
        

    def goal_callback(self, msg):

        # if(self.robot_position == []):
        #     return

        print("Recieved Goal")

        self.goal = [msg.data[0], msg.data[1]]

        new_goal = self.goal

        valid = True
        while(not self.isValid(new_goal)):
            if valid == True:
                print "Not is Valid...Generating New goal: ",
                valid = False
            # print(new_goal)
            new_goal = self.generate_new_goal(self.robot_position, new_goal)

        print(new_goal)
        self.goal = new_goal

        # self.movebase_client(self.goal)
        

    def amcl_pose_callback(self, data):
    
        self.robot_position = [ data.pose.pose.position.x, data.pose.pose.position.x ]
        print(self.robot_position)

    
    def generate_new_goal(self, robot_position, goal_position, step = 0.1):

        x0,y0 = robot_position
        x1,y1 = goal_position

        angle = math.atan( (y1-y0)/(x1-x0) )
        angle2 = math.atan2( (y1-y0),(x1-x0) )
        dist = math.sqrt( (x1-x0)**2 + (y1-y0)**2 )
        x_new, y_new = x0 + (dist - step)*math.cos(angle) ,y0 + (dist - step)*math.sin(angle)   
        print "x0,y0 :" ,x0,y0 
        print "angle atan", angle
        print "angle atan2", angle2
        print "X component :",(dist - step)*math.cos(angle)
        return x_new, y_new
        
if __name__=="__main__":

    new_goal = SendGoal()

    print("Hello")



    



