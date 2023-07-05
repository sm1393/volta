#!/usr/bin/python

import rospy
import math
from turtle import distance
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from volta_navigation.srv import Goal, GoalResponse

class SendGoal:

    def __init__(self):

        self.robot_position = [ 0, 0]
        rospy.init_node("send_goal")
        rospy.Service('goal_server', Goal, self.handle_goal)
        # rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped, self.amcl_pose_callback )
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # UR:(1.499, -1.325, 0.000)
        # UL:(1.623, 1.292, 0.000)
        # LL:(-1.148, 1.261, 0.000)
        # LR:(-1.187, -1.354, 0.000)
        

        self.x_max =  1.499 - 0.10
        self.x_min = -1.187 + 0.20
        
        self.y_max =  1.292 - 0.20
        self.y_min = -1.325 + 0.20


        self.goal = []
        rospy.spin()

    def movebase_client(self):

        
        print("Waiting for server")
        self.client.wait_for_server()
        print("MoveBase Server Up!")

        x,y = self.goal

        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose.position.x = x
        move_base_goal.target_pose.pose.position.y = y

        #  (0, 0, sin(theta/2), cos(theta/2)).

        if ( (x <= self.x_max) and (x >= self.x_max -0.2)  ) :
            # center
            move_base_goal.target_pose.pose.orientation.z = 0.0
            move_base_goal.target_pose.pose.orientation.w = 1.0
        
        elif ( (y <= self.y_max) and (y >= self.y_max -0.2)  ) :
            # left
            move_base_goal.target_pose.pose.orientation.z = 0.7
            move_base_goal.target_pose.pose.orientation.w = 0.7

        elif ( (y >= self.y_min) and (y <= self.y_max - 0.2)  ) :
            # right
            move_base_goal.target_pose.pose.orientation.z = -0.7
            move_base_goal.target_pose.pose.orientation.w = 0.7

        

        self.client.send_goal(move_base_goal)

        self.client.wait_for_result()

        if self.client.get_state() == GoalStatus.SUCCEEDED:
            result = self.client.get_result()
            return 0 
        elif self.client.get_state() == GoalStatus.PREEMPTED:
            return 1
        else:
            return 2

        #  self.client.get_result()


    def isValid(self):
        x,y = self.goal
        if ( x < self.x_max and x > self.x_min ) and ( y < self.y_max and y > self.y_min ) :
            return True

        return False
        

    def handle_goal(self, req):

        print("Recieved Goal:[ %s, %s ]" %(req.x,req.y))
        print "Robot is currently at :",self.robot_position 

        # return 1234

        # if(self.robot_position == []):
        #     return

        # print("Recieved Goal:[%s, %s]" %(req.x,req.y))

        self.goal = [req.x, req.y]

        valid = True
        while(not self.isValid()):
            if valid == True:
                print "Not is Valid...Generating New goal: ",
                valid = False
            self.generate_new_goal()

        print "New Goal :%s"%(self.goal)
        print "Robot is now at :",self.robot_position 


        return GoalResponse(self.movebase_client())


    def amcl_pose_callback(self, data):
        self.robot_position = [ data.pose.pose.position.x, data.pose.pose.position.x ]
        

    
    def generate_new_goal(self, step = 0.1):

        # x0,y0 = self.robot_position
        # x1,y1 = self.goal

        # angle = math.atan2( (y1-y0),(x1-x0) )
        # dist = math.sqrt( (x1-x0)**2 + (y1-y0)**2 )
        # x_new, y_new = x0 + (dist - step)*math.cos(angle) ,y0 + (dist - step)*math.sin(angle)   

        x,y = self.goal
        
        if x > self.soft_xmax :
            x = self.soft_xmax
        elif x < self.soft_xmin :
            x = self.soft_xmin

        if y > self.soft_ymax :
            y = self.soft_ymax
        elif y < self.soft_ymin :
            y = self.soft_ymin
        
        self.goal = [ x, y ]
        
if __name__=="__main__":

    new_goal = SendGoal()

    print("Hello")



    



