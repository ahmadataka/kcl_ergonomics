#!/usr/bin/env python
import roslib
roslib.load_manifest('kcl_ergonomics')
import rospy
import tf
import readchar
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class keyboard_joy(object):
  def __init__(self):
    rospy.init_node('keyboard_joy')
    self.key_pub = rospy.Publisher('keyboard_msg', Joy, queue_size = 10)
    key_sub = rospy.Subscriber('keyboard_input', String, self.get_key)
    rate = rospy.Rate(40.0)
    self.key_read = String()
    self.flag = 0
    while not rospy.is_shutdown():
	#if(self.flag == 0):
	  #self.key_read.data = '?'
	self.key_joy = Joy()
	#self.transform_to_send()

        rate.sleep()

  def get_key(self,msg):
    self.key_read.data = msg.data
    self.flag = 1
    print(self.key_read)
    self.transform_to_send()

  def transform_to_send(self):
    #self.key_joy.header.stamp = rospy.get_time()
    #self.key_joy.header.frame_id = 0
    self.key_joy.axes = []
    self.key_joy.buttons = []
    self.axes = [0, 0, 0, 0, 0, 0]
    self.buttons = [0, 0, 0, 0]

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

    for i in range(0, 6):
      self.key_joy.axes.append(self.axes[i])
    for i in range(0, 4):
      self.key_joy.buttons.append(self.buttons[i])
    self.flag = 0
    self.key_pub.publish(self.key_joy)

if __name__ == '__main__':
    try:
        keyboard_joy()
    except rospy.ROSInterruptException: pass
