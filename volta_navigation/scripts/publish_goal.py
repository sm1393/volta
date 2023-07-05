#!/usr/bin/python

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

global goals_list

goals_list = [[0,0] ,[2,0] , [2,2] ]



def response_callback(msg):

    global goals_list

    if len(goals_list)==0:
        print "No more Goals!"

    goal_publisher = rospy.Publisher('/goals', Pose, queue_size=1, latch=True)

    goal = Pose()
    goal.position.x = goals_list[0][0]
    goal.position.y = goals_list[0][1]
    
    goal_publisher.publish(goal)

    goals_list.pop(0)

if __name__=="__main__":


    rospy.init_node("test_goal_pub")
    rospy.Subscriber('/response', Bool, response_callback)

    goal_publisher = rospy.Publisher('/goals', Pose, queue_size=1)

    goal = Pose()
    goal.position.x = goals_list[0][0]
    goal.position.y = goals_list[0][1]
    
    goal_publisher.publish(goal)

    rospy.spin()



    


