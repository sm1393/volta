# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import os
# import numpy as np

# class Nodo(object):
#     def __init__(self):
#         # Params
#         self.image = None
#         self.br = CvBridge()
#         # Node cycle rate (in Hz).
#         self.loop_rate = rospy.Rate(1)

#         # Publishers
#         self.pub = rospy.Publisher('imagetimer', Image,queue_size=1)

#         # Subscribers
#         rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

#     def callback(self, msg):
#         rospy.loginfo('Image received...')
#         self.image = self.br.imgmsg_to_cv2(msg)
#         print(self.image.shape)
#         cv2.imshow('Display', self.image)
      
#         # the 'q' button is set as the
#         # quitting button you may use any
#         # desired button of your choice
#         cv2.waitKey(0)


#     def start(self):
#         rospy.loginfo("Timing images")
#         #rospy.spin()
#         while not rospy.is_shutdown():
#             rospy.loginfo('publishing image')
#             br = CvBridge()
#             if self.image is not None:
#                 self.pub.publish(br.cv2_to_imgmsg(self.image))
#             self.loop_rate.sleep()

# if __name__ == '__main__':
#     rospy.init_node("imagetimer111", anonymous=True)
#     my_node = Nodo()
#     my_node.start()


#!/usr/bin/env python2.7
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Print "Hello!" to terminal
print "Hello!"

# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('opencv_example', anonymous=True)

# Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
rospy.loginfo("Hello ROS!")

# Initialize the CvBridge class
bridge = CvBridge()

# Define a function to show the image in an OpenCV Window
def show_image(img):
  cv2.imshow("Image Window", img)
  cv2.waitKey(1)
  cv2.destroyAllWindows()

# Define a callback for the Image message
def image_callback(img_msg):
  # log some info about the image topic
  rospy.loginfo(img_msg.header)

  # Try to convert the ROS Image message to a CV2 Image
  try:

      cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
      print(cv_image.shape)
  except CvBridgeError, e:
      rospy.logerr("CvBridge Error: {0}".format(e))

  # Flip the image 90deg
  # cv_image = cv2.transpose(cv_image)
  # cv_image = cv2.flip(cv_image,1)

  # Show the converted image
  show_image(cv_image)

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

# Initialize an OpenCV Window named "Image Window"
cv2.namedWindow("Image Window", 1)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
  rospy.spin()

