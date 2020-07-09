#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import time
import xlwt
import rospy
import std_msgs
import threading
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from hongfu_bms_msg.msg import HongfuStatus

BMS_V = None
BMS_I = None
CMD_V = None

def BMSCallBack(msg):
  global BMS_V, BMS_I
  BMS_V = msg.Voltage
  BMS_I = msg.Current

def CMDCallBack(msg):
  global CMD_V
  if msg.twist.twist.linear.x > 0.1:
    CMD_V = msg.twist.twist.linear.x
  else:
    CMD_V = 0.0

def loop():
  global BMS_V, BMS_I, CMD_V
  r = rospy.Rate(2)
  workbook = xlwt.Workbook(encoding = 'ascii')
  worksheet = workbook.add_sheet('sheet_1')
  worksheet.write(0, 0, 'Voltage')
  worksheet.write(0, 1, 'Current')
  worksheet.write(0, 2, 'Vx')
  worksheet.write(0, 3, 'Vx')
  i = 1
  while not rospy.is_shutdown():
    if CMD_V:
      worksheet.write(i, 0, BMS_V)
      worksheet.write(i, 1, BMS_I)
      worksheet.write(i, 2, CMD_V)
      worksheet.write(i, 3, BMS_V*BMS_I)
      i = i + 1 
      print ("power: %.3f") % (BMS_V*BMS_I)
    r.sleep()
  filename = "./" + time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime()) + ".xls"
  workbook.save(filename)

def main():
  rospy.init_node("power_test")
  sub_cmd = rospy.Subscriber("/odom", Odometry, CMDCallBack)
  sub_bms = rospy.Subscriber("/hongfu_bms_status_node/hongfu_bms", HongfuStatus, BMSCallBack)

  t = threading.Thread(target=loop)
  t.start()
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
