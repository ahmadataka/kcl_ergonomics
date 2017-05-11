#!/usr/bin/env python
# This is used convert from keyboard input to Joy message
import roslib
roslib.load_manifest('kcl_ergonomics')
import rospy
import tf
import readchar
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class keyboard_joy(object):
  def __init__(self):
    # Initialize the ROS Node
    rospy.init_node('keyboard_joy')
    # Define the ROS Publisher used to send Joy message
    self.key_pub = rospy.Publisher('keyboard_msg', Joy, queue_size = 10)
    # Define the ROS Subscriber used to read the character from keyboard
    key_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/keyboard_input', String, self.get_key)
    # Set the rate to be 40 Hz
    rate = rospy.Rate(40.0)
    # Define the String class
    self.key_read = String()

    while not rospy.is_shutdown():
        # Initialize the Joy class
        self.key_joy = Joy()
        # Sleep until 1/40 s
        rate.sleep()

  # This function is used to get the data from the keyboard
  def get_key(self,msg):
    # Read the data
    self.key_read.data = msg.data
    # Prepare the data to be sent
    self.transform_to_send()

  # This function is used to convert from keyboard to Joy
  def transform_to_send(self):
    # Initialize the axes and buttons of the Joy message
    self.key_joy.axes = []
    self.key_joy.buttons = []
    self.axes = [0, 0, 0, 0, 0, 0]
    self.buttons = [0, 0, 0, 0]

    # Map from keyboard character into a Joy message
    if(self.key_read.data == 'a'):
      self.axes[0] = -1
    elif(self.key_read.data == 'd'):
      self.axes[0] = 1
    else:
      self.axes[0] = 0
    if(self.key_read.data == 'w'):
      self.axes[1] = 1
    elif(self.key_read.data == 's'):
      self.axes[1] = -1
    else:
      self.axes[1] = 0
    if(self.key_read.data == 'z'):
      self.axes[2] = -1
    elif(self.key_read.data == 'x'):
      self.axes[2] = 1
    else:
      self.axes[2] = 0
    if(self.key_read.data == 'q'):
      self.axes[3] = 1
    elif(self.key_read.data == 'e'):
      self.axes[3] = -1
    else:
      self.axes[3] = 0
    if(self.key_read.data == 'j'):
      self.axes[4] = 1
    elif(self.key_read.data == 'l'):
      self.axes[4] = -1
    else:
      self.axes[4] = 0
    if(self.key_read.data == 'i'):
      self.axes[5] = 1
    elif(self.key_read.data == 'k'):
      self.axes[5] = -1
    else:
      self.axes[5] = 0
    if(self.key_read.data == 'u'):
      self.buttons[0] = 1
    elif(self.key_read.data == 'o'):
      self.buttons[2] = 1
    elif(self.key_read.data == 'n'):
      self.buttons[1] = 1
    elif(self.key_read.data == 'm'):
      self.buttons[3] = 1

    # Put the value into array
    for i in range(0, 6):
      self.key_joy.axes.append(self.axes[i])
    for i in range(0, 4):
      self.key_joy.buttons.append(self.buttons[i])

    # Publish the data
    self.key_pub.publish(self.key_joy)

if __name__ == '__main__':
    try:
        keyboard_joy()
    except rospy.ROSInterruptException: pass
