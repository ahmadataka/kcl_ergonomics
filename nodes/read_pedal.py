#!/usr/bin/env python
# The code is used to read the pedal status from Serial.

import roslib; roslib.load_manifest('kcl_ergonomics')
import math
import rospy
import serial
import string
from std_msgs.msg import Int32MultiArray

class read_wrist(object):
  def __init__(self):
    # Initialize the ROS Node
    rospy.init_node('read_wrist')
    # Initialize the serial class
    ser = serial.Serial()
    # Define the Serial port
    ser.port = "/dev/ttyUSB0"
    # Set the baudrate
    ser.baudrate = 38400
    # Set the timeout
    ser.timeout = 1
    # Open the serial
    ser.open()
    # Define the variable used to send the pedal status
    pedal_sent = Int32MultiArray()

    # Initialize the flag used to check the availability of the serial data
    flag_sent = 0
    # Define the ROS Publisher
    pedal_pub = rospy.Publisher('pedal_status', Int32MultiArray, queue_size = 10)
    # Set the rate to be 40 Hz
    r = rospy.Rate(40.0)
    while not rospy.is_shutdown():
        # Read the serial
        data = ser.readline()
        # Split the serial data into several words separated by space
        words = data.split()
        # Get the data length
        word_len = len(words)
        # Check whether the data is available
        if(word_len>0):
            # Only listen to '1' or '0' value
            if(words[0]=='1' or words[0]=='0'):
                 # Clear the array and put the data as integer in the array
                 pedal_sent.data = []
                 pedal_sent.data.append(int(words[0]))
                 # Set the flag
                 flag_sent = 1
        # When the flag is set, publish the pedal status
        if(flag_sent == 1):
            pedal_pub.publish(pedal_sent)

if __name__ == '__main__':
    try:
        read_wrist()
    except rospy.ROSInterruptException: pass
