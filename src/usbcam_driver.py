#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Matthew Klein
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the names of the authors nor the names of their
# affiliated organizations may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class usbcam_driver(object):
  def __init__(self):
    #ROS init
    rospy.init_node('usbcam_driver')
    self.imPub = rospy.Publisher('/camera/image',Image,queue_size=1)
    self.cap = cv2.VideoCapture(0) # create video capture object
    self.bridge = CvBridge() # Create bridge between OpenCV and ROS

  def pubImg(self):
    cvImage = self.cap.read()
    rosImageMsg = self.bridge.cv2_to_imgmsg(cvImage[1],encoding="bgr8")
    rosImageMsg.header.stamp = rospy.get_rostime()
    self.imPub.publish(rosImageMsg)

  def run(self):
    while not rospy.is_shutdown():
      self.pubImg()

if __name__ == "__main__":
  cam_driver = usbcam_driver()
  cam_driver.run()
