#!/usr/bin/env python
# The code is used to read the wrist's quarternion data from the Serial.
# Created by King's College London and Queen Mary University of London, 2017.

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
    # Initialize the ROS Node
    rospy.init_node('read_wrist')

    # Initialize the serial class
    ser = serial.Serial()
    # Define the serial port
    ser.port = "/dev/rfcomm0"
    # Set the baudrate
    ser.baudrate = 115200
    # Set the timeout limit
    ser.timeout = 1
    # Open the serial
    ser.open()

    # Initialize the quaternion class for the first sensor
    quat_sent = Quaternion()
    # Initialize the quaternion class for the second sensor
    quat_ref_sent = Quaternion()

    # Initialize the flag used to check the availability of the received serial data
    flag_sent = 0

    # Define the ROS Publisher for both wrist sensors
    quat_ref_pub = rospy.Publisher('wrist_ref_quaternion', Quaternion, queue_size = 10)
    quat_pub = rospy.Publisher('wrist_quaternion', Quaternion, queue_size = 10)
    # Set the rate
    r = rospy.Rate(40.0)
    while not rospy.is_shutdown():
        # Read data from the Serial
        data = ser.readline()
        # Split the serial data into several words separated by space
        words = data.split()
        # Get the length of the data array
        word_len = len(words)

        # Check whether the serial data contains the correct length (in this case 16)
        if word_len == 16:
            # Set the flag to indicate the serial data has been received
            flag_sent = 1

            # Try to read the serial data
            try:
                # Read the first sensor's quaternion
                quat_ref_sent.w = float(words[1])
                quat_ref_sent.x = float(words[3])
                quat_ref_sent.y = float(words[5])
                quat_ref_sent.z = float(words[7])
                # Read the second sensor's quaternion
                quat_sent.w = float(words[9])
                quat_sent.x = float(words[11])
                quat_sent.y = float(words[13])
                quat_sent.z = float(words[15])

            except ValueError:
                continue

        # Publish both quaternions
        quat_pub.publish(quat_sent)
        quat_ref_pub.publish(quat_ref_sent)


if __name__ == '__main__':
    try:
        read_wrist()
    except rospy.ROSInterruptException: pass
