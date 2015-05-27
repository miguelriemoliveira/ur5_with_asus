#!/usr/bin/env python  

from subprocess import call
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('robot_calibration')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    rate.sleep()

    #Calibrate origin
    raw_input("Take the ee_link of the robot to the ORIGIN POINT and press a key when done")
    rate.sleep()
    try:
        (trans,rot) = listener.lookupTransform('/world', '/ee_link', rospy.Time(0))
        rospy.set_param('/ur5_with_asus_calibration/calibration_origin', [trans[0], trans[1], trans[2]])
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.loginfo("Error")
    
    #Calibrate X Axis
    raw_input("Take the ee_link of the robot to the X AXIS POINT and press a key when done")
    rate.sleep()
    try:
        (trans,rot) = listener.lookupTransform('/world', '/ee_link', rospy.Time(0))
        rospy.set_param('/ur5_with_asus_calibration/calibration_xaxis', [trans[0], trans[1], trans[2]])
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.loginfo("Error")

    #Calibrate Y Axis
    raw_input("Take the ee_link of the robot to the Y AXIS POINT and press a key when done")
    rate.sleep()
    try:
        (trans,rot) = listener.lookupTransform('/world', '/ee_link', rospy.Time(0))
        rospy.set_param('/ur5_with_asus_calibration/calibration_yaxis', [trans[0], trans[1], trans[2]])
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.loginfo("Error")

    resp = raw_input("would you like to dump new parameters to three_point.yaml file? [y | N]")
    print resp
    #dump parameters to file

    if (resp=="y" or resp=="Y"):
        rospy.loginfo("Dumping parameters to file three_points.yaml. If you agree copy the file to calibration directory")
        call(["rosparam dump three_points.yaml /ur5_with_asus_calibration -v",""], shell=True)


