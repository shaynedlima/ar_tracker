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

# GLOBAL VARIABLES
marker_xOffset = 0.05 #cm

def callback(data):
    #print data.markers[0]
    #print "Orientation: " + str(data.markers[0].pose.pose.orientation)
    markers = data.markers

    tempPoint = []
    

    goalRegions = MarkerArray()
    goalRegions.markers = []

    tf_listener = tf.TransformListener()
    time.sleep(5)


    if(len(markers) == 3):
        for i in range(0,3):
            if(markers[i].id == 0):
                # Red marker
                p1 = markers[i] 
            if(markers[i].id == 1):
                # Blue marker
                p2 = markers[i]
            #if(markers[i].id == 2):
                # Calibration marker near base
                #calibrationMarker = markers[i]
       
        points = [p1, p2]

        trans = []
        rot = []
        
    

        for i in range(0,2):
            #tempPoint.append(geometry_msgs.msg.PointStamped())
            tempPoint.append(geometry_msgs.msg.PoseStamped())

            tempPoint[i].header.frame_id = "camera_link"
            #tempPoint[i].header.stamp = rospy.Time(0)
            tempPoint[i].pose.position.x = points[i].pose.pose.position.x
            tempPoint[i].pose.position.y = points[i].pose.pose.position.y
            tempPoint[i].pose.position.z = points[i].pose.pose.position.z

            tempPoint[i].pose.orientation.x = points[i].pose.pose.orientation.x
            tempPoint[i].pose.orientation.y = points[i].pose.pose.orientation.y
            tempPoint[i].pose.orientation.z = points[i].pose.pose.orientation.z
            tempPoint[i].pose.orientation.w = points[i].pose.pose.orientation.w


            # Finding goal region locations relative to calibration marker
            tempPoint[i] = tf_listener.transformPose("ar_marker_2",tempPoint[i])
            #tempPoint[i].point.x = tempPoint[i].point.x - marker_xOffset
            

            # Goal region markers
            goal = Marker()
            goal.header.frame_id = "/ar_marker_2"
            goal.header.stamp = rospy.Time()
            #goal.header.frame_id = "/base_link"
            goal.type = Marker.CYLINDER
            goal.action = Marker.ADD
            goal.id = i
            goal.pose.position.x = tempPoint[i].pose.position.x
            goal.pose.position.y = tempPoint[i].pose.position.y
            goal.pose.position.z = tempPoint[i].pose.position.z
            goal.scale.x = 0.3
            goal.scale.y = 0.3
            goal.scale.z = 0.005
            goal.color.a = 1.0
            goal.color.g = 0.0


            goal.pose.orientation.w = 1.0

            if(points[i].id == 0): #Red
                print "RED"
                goal.color.r = 1.0
                goal.color.b = 0.0
            if(points[i].id == 1): #Blue
                print "BLUE"
                goal.color.b = 1.0
                goal.color.r = 0.0

            goalRegions.markers.append(goal)
    
        print "Goal Regions length= " + str(len(goalRegions.markers))
        print goalRegions
        publisher.publish(goalRegions)

        # Position of point relative to base_link
        #x_base = 1
        #y_base = 1
        #z_base = 0
        #print "p1"
        #print p1
    
    #istener.transformPose(p, "/ar_marker_0")
    #print p
    #t = tf.Transformer(True, rospy.Duration(10.0))
    #m = robot_msgs.msg.TransformStamped()
    #m.header.frame_id = "ar_marker_frame"
    #m.parent_id = "PARENT"

    #m = tf.transformPose(marker.pose)
    



    #transform = self.tf_buffer.lookup_transform("robot_center", "base_front_laser_link_horizontal",
                                                #rospy.Time(0),
                                                #rospy.Duration(10))

    
    #print tf_listener.frameExists("ar_marker_0")

    
    # A point relative to the camera frame
    #try:
    #print "tried"


    #testPoint2 = geometry_msgs.msg.PoseStamped()
    #testPoint2.header.frame_id = "camera_link"
    #testPoint2.header.stamp = rospy.Time(0)
    #testPoint2.pose.position.x = 0.0
    #testPoint2.pose.position.y = 0.0
    #testPoint2.pose.position.z = 0.0

    #testPoint2.pose.orientation.x = 0.0
    #testPoint2.pose.orientation.y = 0.0
    #testPoint2.pose.orientation.z = 0.0
    #testPoint2.pose.orientation.w = 0.0

    #p2 = tf_listener.transformPose("ar_marker_0",testPoint2)

    # To translate to base_link frame subtract known constant to x-value of point

        
    #print p2

    



    
def listener():
    rospy.init_node('listener', anonymous=True)
    
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    publisher = rospy.Publisher("goalRegions", MarkerArray, queue_size=1)
    listener()



