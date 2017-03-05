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
from cv_bridge import CvBridge

from os.path import expanduser
from os.path import isdir
from os import mkdir
import cv2

class pic_to_disk(object):
  def __init__(self):
    #Init ROS
    rospy.init_node('pic_to_disk')

    topic_in  = rospy.get_param('~topic_in','/camera/image')
    rospy.Subscriber(topic_in, Image, self.imSubCB)

    #Set the path of the folder where the images will be stored
    self.imPath = rospy.get_param('~path',"{0}{1}".format(expanduser('~'),'/pic_to_disk/'))
    #If the directory doesn't exist, make it so cv2.imwrite can write to it
    if not isdir(self.imPath):
      mkdir(self.imPath)

    #Create bridge to go between ROS messages and openCV images
    self.bridge = CvBridge()
    
  def imSubCB(self, msg):
    #Convert the ROS Image message to a cv2 image object
    cvImage = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    #Create the file name with the format seconds-nanoseconds
    imFileName = "{0}{1}-{2}.jpg".format(self.imPath,msg.header.stamp.secs,msg.header.stamp.nsecs)
    #And write it to disk
    cv2.imwrite(imFileName,cvImage)

  #self.run just runs a while loop allowing the subscriber to call imSubCB
  def run(self):
    rospy.spin()

if __name__ == "__main__":
  p2d = pic_to_disk()
  p2d.run()
