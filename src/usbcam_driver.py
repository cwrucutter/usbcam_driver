#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import cv2
import sys

if __name__ == "__main__":
  #ROS init
  rospy.init_node('usbcam_driver')
  imPub = rospy.Publisher('/camera/image', Image, queue_size=1)
  
  cap = cv2.VideoCapture(0) # Create video capture object
  bridge = CvBridge() # Create bridge between OpenCV and ROS

  while not rospy.is_shutdown():
    cvImage = cap.read()
    rosImageMsg = bridge.cv2_to_imgmsg(cvImage[1], encoding="bgr8")
    rosImageMsg.header.stamp = rospy.get_rostime()
    imPub.publish(rosImageMsg)
