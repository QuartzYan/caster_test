#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys

import rospy 
import std_msgs

import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
sub = None
pub = None

def callBack(msg):
  try:
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
  except CvBridgeError as e:
    print(e)

  flipped = cv2.flip(cv_image, 0)
  
  try:
    pub.publish(bridge.cv2_to_imgmsg(flipped, "bgr8"))
  except CvBridgeError as e:
    print(e)

def main():
  global sub, pub
  rospy.init_node("image_transform")
  sub = rospy.Subscriber("raw_image", Image, callBack)
  pub = rospy.Publisher("transform_image", Image, queue_size=1)
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass