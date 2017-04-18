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
import tf

class path_broadcaster(object):
    def __init__(self):
        rospy.init_node('wrist_broadcaster')
        self.index = rospy.get_param('/human')
        tip_sub = rospy.Subscriber('wrist_quaternion', Quaternion, self.handle_pose)
        tip_ref_sub = rospy.Subscriber('wrist_ref_quaternion', Quaternion, self.handle_pose_ref)
        listener = tf.TransformListener()
        listener2 = tf.TransformListener()
        # while not rospy.is_shutdown():
            # try:
            #     now = rospy.Time.now()
            #     (self.trans,rot) = listener.lookupTransform('torso', 'left_hand_'+str(self.index), rospy.Time(0))
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     continue

            # try:
            #     now = rospy.Time.now()
            #     (self.trans2,rot2) = listener2.lookupTransform('left_elbow_1', 'left_hand_'+str(self.index), rospy.Time(0))
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     continue

        rospy.spin()

    def handle_pose(self, msg):
        path_frame = tf.TransformBroadcaster()
        # path_frame.sendTransform((self.trans[0],self.trans[1],self.trans[2]),
        #                      (msg.x, msg.y, msg.z, msg.w),
        #                      rospy.Time.now(),
        #                      "wrist",
        #                      "torso")
        path_frame.sendTransform((0.0,0.0,0.0),
                                 (msg.x, msg.y, msg.z, msg.w),
                                 rospy.Time.now(),
                                 "wrist","torso")
                                #  "wrist_trial")
        # WARNING: The x of quaternion should be multiplied by (-1)
    def handle_pose_ref(self, msg):
        path_frame2 = tf.TransformBroadcaster()
        # path_frame.sendTransform((self.trans[0],self.trans[1],self.trans[2]),
        #                      (msg.x, msg.y, msg.z, msg.w),
        #                      rospy.Time.now(),
        #                      "wrist",
        #                      "torso")
        path_frame2.sendTransform((0.0,0.0,0.0),
                                 (msg.x, msg.y, msg.z, msg.w),
                                 rospy.Time.now(),
                                 "wrist_ref","torso")
                                #  "wrist_trial")
        # WARNING: The x of quaternion should be multiplied by (-1)


        # print(self.trans)
        # print(self.trans2)


if __name__ == '__main__':
    try:
        path_broadcaster()
    except rospy.ROSInterruptException: pass
