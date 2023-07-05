#!/usr/bin/env python3
import websocket
import time
import json
from signal import signal, SIGINT
import sys
import os

#ROS Dependencies
import rospy
from geometry_msgs.msg import Twist

URL="ws://"+"192.168.43.124"+":7000"

# def signal_handler(signal_received, frame):
#     print("Exiting Flask Server")
#     rospy.signal_shutdown("Keyboard Interrupt") 
#     sys.exit()

def signal_handler(signal_received, frame):
    print("Exiting Glove Client")
    sys.exit()

signal(SIGINT,signal_handler)

class valve_client:
    def __init__(self):
        self.url = URL
        self.ws_obj=self.establish_connection()
        self.x_coord = 0.0
        self.y_coord = 0.0
        self.max_linear_velocity = 0.7
        self.max_angular_velocity = 0.7
        rospy.init_node('valve_index_listener',anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('joy/cmd_vel',Twist, queue_size=10)
        self.rate = rospy.Rate(10)
    
    def establish_connection(self):
        print("Establishing connection")
        conn=False
        while conn is False:
            try:
                #obj=websocket.create_connection(self.url,timeout=1)
                obj=websocket.create_connection(self.url)
                print("Connection Established")
                conn=True
            except:
                print("Trying to establish connection to gloves")
                time.sleep(1)
        return obj

    def get_controller_data(self):
        try:
            self.ws_obj.send("Ack")
            data=json.loads(self.ws_obj.recv())
            return data
        except Exception as e:
            print(e)
            self.ws_obj=self.establish_connection()

    def teleop_bot(self):
        while True:
            try:
                data=self.get_controller_data()
                if data!=None:
                    if data['flag']:
                        self.x_coord = data['trackpad_x']
                        self.y_coord = data['trackpad_y']
                        print(self.x_coord,self.y_coord)
                        self.publish_cmd_vel()
            except Exception as e:
                print("Error occured: ",e)
    
    def publish_cmd_vel(self):
            '''
            Publishes cmd_vel to /joy/cmd_vel
            '''
            move_cmd = Twist()
            move_cmd.linear.x = self.y_coord * self.max_linear_velocity
            move_cmd.angular.z = -self.x_coord * self.max_angular_velocity
            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()

if __name__=="__main__":
    valve_obj= valve_client()
    valve_obj.teleop_bot()

