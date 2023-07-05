#!/usr/bin/python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospy

def make_point(x,y):
    # make a visualization marker array for the occupancy grid
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = 'map'
    m.header.stamp = rospy.Time(0)
    m.ns = 'marker_test_%d' % Marker.POINTS
    m.id = 0
    m.type = Marker.POINTS
    m.points.append(Point(x,y,0))
    m.pose.position.x = x
    m.pose.position.y = y
    m.scale.x = 0.2
    m.scale.y = 0.2
    m.color.r = 1.0;
    m.color.g = 0;
    m.color.b = 0;
    m.color.a = 1.0;
    return m

def marker_publish(x,y, pub):
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = 'map'
    m.header.stamp = rospy.Time(0)
    m.ns = 'marker_test_%d' % Marker.ARROW
    m.id = 0
    m.type = Marker.ARROW
    m.pose.position.x = x
    m.pose.position.y = y
    m.scale = Vector3(0.3,0.06,0.06)
    m.color.r = 0
    m.color.g = 0
    m.color.b = 0
    m.color.a = 1.0
    return m

if __name__=="__main__":

    rospy.init_node("visualization_node")

    marker_pub = rospy.Publisher('goal_server_goal', Marker , queue_size=1, latch=True)
    marker_pub2 = rospy.Publisher('recieved_goal', Marker , queue_size=1, latch=True)
    points_pub = rospy.Publisher('soft_limit', Marker , queue_size=1, latch=True)


    points_pub.publish(make_point(0,0))

    
       

