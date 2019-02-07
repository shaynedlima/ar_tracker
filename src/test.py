#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
#from geometry_msgs.msg import Twist
import geometry_msgs.msg
import math
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import tf
import time
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import statistics

# GLOBAL VARIABLES
marker_xOffset =  -0.805#m -0.305
marker_yOffset = -0.005 #0.035 #m #-0.005
marker_zOffset = -0.015 #m

def callback(data):
    markers = data.markers
    tf_listener = tf.TransformListener()
    t = rospy.Time.now()
    #tf_listener.waitForTransform("/camera_link", "/ar_marker_2", t, rospy.Duration(5.0))
    time.sleep(5)


    x = []
    y = []
    z = []

    q0 = []
    q1 = []
    q2 = []
    q3 = []

    #if(len(markers) == 1 and markers[0].id==2):
       #t = tf_listener.getLatestCommonTime("/base_link", "/map")
    for i in range(0,200):
        (translation,rotation) = tf_listener.lookupTransform("/ar_marker_2", "/kinect2_rgb_optical_frame", rospy.Time(0))

        x.append(translation[0])
        y.append(translation[1])
        z.append(translation[2])

        q0.append(rotation[0])
        q1.append(rotation[1])
        q2.append(rotation[2])
        q3.append(rotation[3])

    x = statistics.median(x)
    y = statistics.median(y)
    z = statistics.median(z)
    q0 = statistics.median(q0)
    q1 = statistics.median(q1)
    q2 = statistics.median(q2)
    q3 = statistics.median(q3)

    euler = tf.transformations.euler_from_quaternion([q0,q1,q2,q3])
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    print "Calibration Values:"
    print "<arg name=\"x_cal\" value=\"" + str(x + marker_xOffset) + "\"/>"
    print "<arg name=\"y_cal\" value=\"" + str(y + marker_yOffset) + "\"/>"
    print "<arg name=\"z_cal\" value=\"" + str(z + marker_zOffset) + "\"/>"

    print "<arg name=\"r_cal\" value=\"" + str(roll) + "\"/>"
    print "<arg name=\"p_cal\" value=\"" + str(pitch) + "\"/>"
    print "<arg name=\"yaw_cal\" value=\"" + str(yaw) + "\"/>"



def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    publisher = rospy.Publisher("goalRegions", MarkerArray, queue_size=1)
    listener()
