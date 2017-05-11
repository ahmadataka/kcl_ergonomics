#!/usr/bin/env python
# The code is used to broadcast 2 wrist sensor into TF frames .

import roslib; roslib.load_manifest('kcl_ergonomics')
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import tf

class path_broadcaster(object):
    def __init__(self):
        # Initialize the ROS Node
        rospy.init_node('wrist_broadcaster')
        # Get the human index from the Kinect
        self.index = rospy.get_param('/human')
        # Define the subscriber of the quaternion value from both sensors
        tip_sub = rospy.Subscriber('wrist_quaternion', Quaternion, self.handle_pose)
        tip_ref_sub = rospy.Subscriber('wrist_ref_quaternion', Quaternion, self.handle_pose_ref)

        # Spin and wait for the subscribed data
        rospy.spin()

    def handle_pose(self, msg):
        # Initialize the broadcaster
        path_frame = tf.TransformBroadcaster()
        # Broadcast the first sensor frame
        path_frame.sendTransform((0.0,0.0,0.0),
                                 (msg.x, msg.y, msg.z, msg.w),
                                 rospy.Time.now(),
                                 "wrist","torso")
        # WARNING: The x of quaternion should be multiplied by (-1)

    def handle_pose_ref(self, msg):
        # Initialize the broadcaster
        path_frame2 = tf.TransformBroadcaster()
        # Broadcast the second sensor frame
        path_frame2.sendTransform((0.0,0.0,0.0),
                                 (msg.x, msg.y, msg.z, msg.w),
                                 rospy.Time.now(),
                                 "wrist_ref","torso")
        # WARNING: The x of quaternion should be multiplied by (-1)


if __name__ == '__main__':
    try:
        path_broadcaster()
    except rospy.ROSInterruptException: pass
