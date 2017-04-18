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
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64MultiArray
import tf

class read_wrist(object):
  def __init__(self):
    rospy.init_node('read_wrist')
    ser = serial.Serial()
    # ser.port = "/dev/ttyUSB0"
    ser.port = "/dev/rfcomm0"
    # ser.baudrate = 38400
    ser.baudrate = 115200
    ser.timeout = 1
    ser.open()
    quat_sent = Quaternion()
    quat_ref_sent = Quaternion()
    euler_sent = Float64MultiArray()
    (r, p, y) = (0.0, 0.0, 0.0)
    flag_sent = 0
    quat_ref_pub = rospy.Publisher('wrist_ref_quaternion', Quaternion, queue_size = 10)
    quat_pub = rospy.Publisher('wrist_quaternion', Quaternion, queue_size = 10)
    # euler_pub = rospy.Publisher('wrist_euler', Float64MultiArray, queue_size = 10)
    r = rospy.Rate(40.0)
    while not rospy.is_shutdown():
        data = ser.readline()
        # print data
        words = data.split()
        word_len = len(words)
        # print word_len
        if word_len == 16:
            flag_sent = 1
            try:
                quat_ref_sent.w = float(words[1])
                quat_ref_sent.x = float(words[3])
                quat_ref_sent.y = float(words[5])
                quat_ref_sent.z = float(words[7])
                quat_sent.w = float(words[9])
                quat_sent.x = float(words[11])
                quat_sent.y = float(words[13])
                quat_sent.z = float(words[15])
                (r, p, y) = tf.transformations.euler_from_quaternion([quat_sent.x, quat_sent.y, quat_sent.z, quat_sent.w])

            except ValueError:
                continue
        euler_sent = Float64MultiArray()
        euler_sent.data = []
        euler_sent.data.append(r)
        euler_sent.data.append(p)
        euler_sent.data.append(y)
        quat_pub.publish(quat_sent)
        quat_ref_pub.publish(quat_ref_sent)
        # if(flag_sent == 1):
        #     euler_pub.publish(euler_sent)
        # r.sleep()
        #rospy.spin()

if __name__ == '__main__':
    try:
        read_wrist()
    except rospy.ROSInterruptException: pass
