#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#


import roslib; roslib.load_manifest('kcl_ergonomics')
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64MultiArray
import tf

class path_broadcaster(object):
    def __init__(self):
        rospy.init_node('euler_wrist')
        euler_pub = rospy.Publisher('wrist_euler', Float64MultiArray, queue_size = 10)
        euler_sent = Float64MultiArray()
        listener = tf.TransformListener()
        rates = rospy.Rate(40.0)
        while not rospy.is_shutdown():
            # self.mode = rospy.get_param('/robot_mode')
            # reference = 'wrist_ref'
            # if(self.mode == 0):
            #     reference = 'wrist_ref'
            # else:
            #     reference = 'wrist_calib'
            try:
                now = rospy.Time.now()
                (self.trans,rot) = listener.lookupTransform('wrist_ref', 'wrist', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            (r, p, y) = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
            euler_sent = Float64MultiArray()
            euler_sent.data = []
            euler_sent.data.append(r)
            euler_sent.data.append(p)
            euler_sent.data.append(y)
            euler_pub.publish(euler_sent)
            rates.sleep()

if __name__ == '__main__':
    try:
        path_broadcaster()
    except rospy.ROSInterruptException: pass
