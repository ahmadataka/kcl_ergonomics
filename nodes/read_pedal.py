#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#


import roslib; roslib.load_manifest('kcl_ergonomics')
import math
import rospy
import serial
import string
from std_msgs.msg import Int32MultiArray

class read_wrist(object):
  def __init__(self):
    rospy.init_node('read_wrist')
    ser = serial.Serial()
    ser.port = "/dev/ttyUSB0"
    # ser.port = "/dev/rfcomm0"
    ser.baudrate = 38400
    # ser.baudrate = 115200
    ser.timeout = 1
    ser.open()
    pedal_sent = Int32MultiArray()

    flag_sent = 0
    pedal_pub = rospy.Publisher('pedal_status', Int32MultiArray, queue_size = 10)
    r = rospy.Rate(40.0)
    while not rospy.is_shutdown():
        data = ser.readline()
        print data

        words = data.split()
        word_len = len(words)
        if(word_len>0):
            # print word_len
            if(words[0]=='1' or words[0]=='0'):
                 pedal_sent.data = []
                 pedal_sent.data.append(int(words[0]))
                #  print int(words[0])
                 flag_sent = 1
        if(flag_sent == 1):
            pedal_pub.publish(pedal_sent)
        # r.sleep()
        #rospy.spin()

if __name__ == '__main__':
    try:
        read_wrist()
    except rospy.ROSInterruptException: pass
