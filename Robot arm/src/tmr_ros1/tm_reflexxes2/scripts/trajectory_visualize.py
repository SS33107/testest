#!/usr/bin/env python 

import rospy
# import math
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

if __name__ == '__main__':
    rospy.init_node('TrajVisualize_node')

    listener = tf.TransformListener() #This will listen to the tf data later

    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=100) 

    marker = Marker() 
    marker.id = 0
    marker.header.frame_id = 'base_link'
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.LINE_STRIP
    marker.ns = 'TrajVisualize_node'
    marker.action = Marker.ADD
    marker.scale.x = 0.01
    marker.pose.orientation.w = 1.0
    marker.color.a = 1.0
    marker.color.g = 1.0

    arrow = Marker() 
    arrow.id = 1
    arrow.header.frame_id = 'base_link'
    arrow.header.stamp = rospy.Time.now()
    arrow.type = Marker.ARROW
    arrow.ns = 'TrajVisualize_node'
    arrow.action = Marker.ADD
    arrow.scale.x = 0.005
    arrow.scale.y = 0.005
    arrow.scale.z = 0.003
    arrow.pose.orientation.w = 1.0
    arrow.color.r = 1.0
    arrow.color.g = 0.0
    arrow.color.b = 0.0
    arrow.color.a = 1.0

    arrow2 = Marker() 
    arrow2.id = 2
    arrow2.header.frame_id = 'base_link'
    arrow2.header.stamp = rospy.Time.now()
    arrow2.type = Marker.ARROW
    arrow2.ns = 'TrajVisualize_node'
    arrow2.action = Marker.ADD
    arrow2.scale.x = 0.005
    arrow2.scale.y = 0.005
    arrow2.scale.z = 0.003
    arrow2.pose.orientation.w = 1.0
    arrow2.color.r = 1.0
    arrow2.color.g = 0.0
    arrow2.color.b = 1.0
    arrow2.color.a = 1.0

    # obstacle : 0.65, -0.186, 0.28

    obstacle = Point()
    obstacle.x = 0.65;
    obstacle.y = -0.186;
    obstacle.z = 0.28;

    goal = Point()
    goal.x = 1;
    goal.y = 0;
    goal.z = 0;
    
    rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('base_link', 'gripper_link', rospy.Time(0)) #gper
            (trans2, rot2) = listener.lookupTransform('base_link', 'joint3_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        newpoint = Point()
        newpoint.x = trans[0]
        newpoint.y = trans[1]
        newpoint.z = trans[2]

        joint3 = Point()
        joint3.x = trans2[0]
        joint3.y = trans2[1]
        joint3.z = trans2[2]

        if len(marker.points) > 700:
            marker.points.pop(0)        # To make the old trail disappear continuously

        marker.points.append(newpoint)

        dist = (obstacle.x - newpoint.x)*(obstacle.x - newpoint.x) + (obstacle.y - newpoint.y)*(obstacle.y - newpoint.y) + (obstacle.z - newpoint.z)*(obstacle.z - newpoint.z)
        
        if dist < 0.09 :
            arrow.points = [newpoint,Point(0.65,-0.186,0.28)]
        else :
            arrow.points = [newpoint,newpoint]

        if joint3.z < 0.25 :
            arrow2.points = [joint3,Point(trans2[0],trans2[1],0)]
        else :
            arrow2.points = [joint3,joint3]
        
        marker_pub.publish(marker)
        #marker_pub.publish(arrow)
        marker_pub.publish(arrow2)

        rate.sleep()


