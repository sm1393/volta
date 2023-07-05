#!/usr/bin/env python
import rospy
# from std_msgs.msg import String
from rosgraph_msgs.msg import Log

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if data.name == "/move_base":
        if "goal reached" in data.msg.lower():
            print("Reached destination")
            # ws_data[index_position] = {
            #     "/move_base": "Reached destination"
            # }
        elif "off" in data.msg.lower():
            print("robot on the way")
        #     ws_data[index_position] = {
        #     "/move_base": "Robot on the way"
        # }
        elif "fail" in data.msg.lower():
            print("robot is stuck")
        #     ws_data[index_position] = {
        #     "/move_base": "Robot is stuck"
        # }
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/rosout", Log, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()