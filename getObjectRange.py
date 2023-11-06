#!/usr/bin/env python

#Takes average of 3 lidar scans of object closest to robot if its is under d+e. Filter out really for objects.
#Also get angle of reading. d* cos and d*sin of Angle gets you AO x and y
import tf
import math
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

ao = Point()
pos = Point()
radius = .35

def odom(odom_data): #Get local pos and twist
        pos.x = odom_data.pose.pose.position.x
        pos.y = odom_data.pose.pose.position.y
        quaternion = [odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        pos.z = euler[2]*180/math.pi
        if pos.z < 0:
                pos.z = pos.z + 360
        pub2.publish(pos)

def distance(laser_data):
        dist=[]
        ranges = laser_data.ranges[:]
        for i in ranges:
                if i <= radius:
                        if i == 0.0:
                                pass
                        else:
                                dist.append(i)
        if not dist:
                ao.x = 999
                ao.y = 999
        else:

                min_dist = np.amin(dist)
                angle = np.where(ranges == min_dist)[0][0] #index of 360 values, so 0-159
                ao.x = np.float64(min_dist)
                ao.y = np.float64(angle)

                dist = []
        
	ao.z = ranges[0]
        pub1.publish(ao)
        rate.sleep()

if __name__=='__main__':
        rospy.init_node('getObjectRange')
        rate = rospy.Rate(5)
        pub1 = rospy.Publisher('object_loc', Point, queue_size=1)
        pub2 = rospy.Publisher('my_pos', Point, queue_size=1)
        rospy.Subscriber('/odom', Odometry, odom)
        rospy.Subscriber('/scan', LaserScan, distance)
        rospy.spin()

