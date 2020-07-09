#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import math
import rospy 
import std_msgs

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class odomTest():
  def __init__(self):
    self.flage = False
    self.first_odom = True
    self.sub_odom = rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.odomCallBack)
    self.sub_flage = rospy.Subscriber("/start_test", Bool, self.flageCallBack)
    self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.spin()

  def odomCallBack(self, msg):
    if(self.flage):
      orientation_q = msg.pose.pose.orientation
      orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
      (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
      if(self.first_odom):
        if msg.pose.pose.position.x > 0.001 or msg.pose.pose.position.y > 0.001 or yaw > 0.001:
          rospy.logwarn("robot not zero pose!! please restart driver!!")
        else:
          self.first_odom = False
      else:
        if msg.pose.pose.position.x > 5.0:
          cmd_msg = Twist()
          self.pub_twist.publish(cmd_msg)
          rospy.loginfo("x:%f y:%f th:%f", msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
          self.flage = False
          self.first_odom = True
        else:
          cmd_msg = Twist()
          cmd_msg.linear.x = 0.3
          self.pub_twist.publish(cmd_msg)

  def flageCallBack(self, msg):
    if(msg.data):
      self.flage = True

def main():
  rospy.init_node("odom_test")
  t = odomTest()
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
