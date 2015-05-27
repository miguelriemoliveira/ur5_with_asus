#!/usr/bin/env python  

import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion

if __name__ == '__main__':
    rospy.init_node('get_ee_link_to_camera_link_tf')

    listener = tf.TransformListener()

    rate = rospy.Rate(10)
    flg = False
    while not rospy.is_shutdown() and not flg:

        rate.sleep()
        rospy.loginfo("Listening to transform from ee_link to camera_link")
        try:
            (trans,rot) = listener.lookupTransform('/ee_link', '/camera_link', rospy.Time(0))
            flg = True
            rospy.loginfo("Origin %s", trans)
            angles = euler_from_quaternion(rot)
            rospy.loginfo("RPY %s", angles)


            rospy.loginfo("Copy this to the publish_fake_tf.launch (inverted roll / yaw)")
            print str(trans[0]) + ", " + str(trans[1]) + ", " + str(trans[2]) + ", "  + str(angles[2]) + ", " + str(angles[1]) + ", " + str(angles[0])

            #rospy.loginfo("Copy this to the xacro file")
            #print str(trans[0]) + ", " + str(trans[1]) + ", " + str(trans[2]) + ", "  + str(angles[2]) + ", " + str(angles[1]) + ", " + str(angles[0])
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Could no get transform from /ee_link to /camera_link")
    


