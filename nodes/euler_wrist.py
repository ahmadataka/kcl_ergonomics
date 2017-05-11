#!/usr/bin/env python
# The code is used to produce the Euler angle between 2 wrist sensors.


import roslib; roslib.load_manifest('kcl_ergonomics')
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64MultiArray
import tf

class path_broadcaster(object):
    def __init__(self):
        # Initialize the ROS Node
        rospy.init_node('euler_wrist')
        # Define the ROS Publisher of the euler angle
        euler_pub = rospy.Publisher('wrist_euler', Float64MultiArray, queue_size = 10)
        # Define the ROS Listener
        listener = tf.TransformListener()
        # Set the Rate to be 40 Hz
        rates = rospy.Rate(40.0)
        while not rospy.is_shutdown():
            # Try to listen to the frame 'wrist' with respect to frame 'wrist_ref'
            try:
                now = rospy.Time.now()
                # Listen to the frame 'wrist' with respect to frame 'wrist_ref'
                (self.trans,rot) = listener.lookupTransform('wrist_ref', 'wrist', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            # Transform the quaternion to euler angle
            (r, p, y) = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
            # Define the class for the euler angle
            euler_sent = Float64MultiArray()
            # Empty the arrat
            euler_sent.data = []
            # Put the Euler angle into the array
            euler_sent.data.append(r)
            euler_sent.data.append(p)
            euler_sent.data.append(y)
            # Publish the euler array
            euler_pub.publish(euler_sent)
            # Sleep until 1/40 s
            rates.sleep()

if __name__ == '__main__':
    try:
        path_broadcaster()
    except rospy.ROSInterruptException: pass
