#!/usr/bin/python2

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import os
import sys
import argparse

import rospy

import cv2
import cv_bridge

from sensor_msgs.msg import (
    Image,
)
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32

class main(object):
  def __init__(self):
    rospy.init_node('rsdk_xdisplay_image', anonymous=True)
    self.mode = rospy.get_param('/number_screen')
    rula_sub = rospy.Subscriber('/fourbythree_topics/ergonomics/rula_score', Float64MultiArray, self.get_rula)
    range_sub = rospy.Subscriber('/out_of_range', Int32, self.get_range)
    initial = rospy.Time.now()
    self.rula = 1
    self.range_stat = 0
    while not rospy.is_shutdown():
        present = rospy.Time.now()

        if(self.rula == 1):
            color = 'violet'
        elif(self.rula==2):
            color = 'indigo'
        elif(self.rula==3):
            color = 'blue'
        elif(self.rula == 4):
            color = 'green'
        elif(self.rula==5):
            color = 'yellow'
        elif(self.rula==6):
            color = 'orange'
        else:
            color = 'red'
        if(self.mode == 0):
            self.send_image(color)
        else:
            self.send_image(1)

    return 0

  def get_rula(self,vec):
    self.rula = vec.data[0]

  def get_range(self,vec):
    # self.abduct_stat = vec.data[0]
    self.range_stat = vec.data

  def send_image(self, show):
      if(self.mode == 0):
          msg = Image()
          msg.height = 600/3
          msg.width = 1024/3
          msg.encoding = "bgr8"
          msg.is_bigendian = 0
          msg.step = msg.width*3
          msg.data = []
          index = 255.0/float(msg.width*msg.height)
          r = rospy.Rate(40.0)
          for i in range(0,msg.width*msg.height):
              colors = int(i*index)

              if(show == 'red'):
                  msg.data.append(0)
                  msg.data.append(0)
                  msg.data.append(255)
              elif(show == 'blue'):
                  msg.data.append(255)
                  msg.data.append(0)
                  msg.data.append(0)
              elif(show == 'yellow'):
                  msg.data.append(0)
                  msg.data.append(255)
                  msg.data.append(255)
              elif(show == 'green'):
                  msg.data.append(0)
                  msg.data.append(255)
                  msg.data.append(0)
              elif(show == 'orange'):
                  msg.data.append(0)
                  msg.data.append(165)
                  msg.data.append(255)
              elif(show == 'violet'):
                  msg.data.append(211)
                  msg.data.append(0)
                  msg.data.append(148)
              elif(show == 'indigo'):
                  msg.data.append(130)
                  msg.data.append(0)
                  msg.data.append(75)

      else:
        #   img = cv2.imread('/home/baxter/'+str(int(self.rula))+'.jpg')
          if(self.range_stat==0):
              img = cv2.imread('/home/baxter/ros_ws/src/kcl_baxter/'+str(int(self.rula))+'.jpg')
          else:
              img = cv2.imread('/home/baxter/ros_ws/src/kcl_baxter/OOR.jpg')
          msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
      pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
      pub.publish(msg)
      # Sleep to allow for image to be published.
    #   r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
