#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
import math
import numpy as np
import tf2_ros
import tf2_geometry_msgs


def callback(data):
    #print data.markers[0]
    #print "Orientation: " + str(data.markers[0].pose.pose.orientation)
    markers = data.markers

    if(len(markers) == 3):
        for i in range(0,3):
            if(markers[i].id == 0):
                p1 = markers[i] 
            if(markers[i].id == 1):
                p2 = markers[i]
            if(markers[i].id == 2):
                p3 = markers[i]
                
       
        points = [p1, p2, p3]
      
        # Position of point relative to base_link
        #x_base = 1
        #y_base = 1
        #z_base = 0

        print p1.pose.pose.position.x
        print p1.pose.pose.position.y
        print p1.pose.pose.position.z
        # Position of point relative to camera
        #x_cam = [0, 0, 0]

        

        x_base = [-0.4, -0.4, -0.8]
        y_base = [0.5, -0.5, 0]
        z_base = [0, 0, 0]

        # Compare three known points relative to base to same points relative to kinect
        temp = []
        temp2 = []
        a = np.array([[x_base[0], x_base[1], x_base[2]], [y_base[0], y_base[1], y_base[2]], [z_base[0], z_base[1], z_base[2]]])
        print a


        for i in range(0, 3):
            b = np.array([[points[i].pose.pose.position.x], [points[i].pose.pose.position.y], [points[i].pose.pose.position.z]])
            print 'B'
            print b
            temp.append(np.linalg.solve(a,b))
            print np.linalg.solve(a,b)
            

        print temp

        for i in range(0, 3):
            temp2.append([temp[0][i], temp[1][i], temp[2][i]])

       
        print temp2

        #print temp2[0]
        #b_to_a = np.array([temp2[0], temp2[1], temp2[2]])

        #print b_to_a
        # a = b_to_a * b
        #print np.matmul(b_to_a, b) 
        #print np.array([[1.2], [1.4], [1.2]])

        # Change of basis vector

        # Matrix:
        # x1 x2 x3          lambda
        # y1 y2 y3      *     mu
        # 1   1  1           tau
        #a = np.matrix()



        distance = math.sqrt(x_cam*x_cam + y_cam*y_cam +z_cam*z_cam)
        print "Distance from camera: " + str(distance)

        print "Position: " + str(data.markers[0].pose.pose.position)
    
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose.p)
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()



