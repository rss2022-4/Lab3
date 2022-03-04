#!/usr/bin/env python2

import rospy
import numpy as np
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from visualization_tools import *
# from wall_follower.msg import coordinate_msg

class LinePublisher:
    #the topics to publish and subscribe to
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    WALL_TOPIC = "/wall"
    SIDE = rospy.get_param("wall_follower/side")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    s = 'r'
    SIDE = -1
    
    def __init__(self):
        #a publisher for our line marker
        self.line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=1)
        
        #a subscriber to get the laserscan data
        rospy.Subscriber(self.SCAN_TOPIC, numpy_msg(LaserScan), self.laser_callback)
        #rospy.Subscriber('lin_reg', coordinate_msg, self.laser_callback)
        
    def laser_callback(self, data):
        # x and y should be points on your detected wall
        # here we are just plotting a parabola as a demo
        pts = self.get_important_points(data)
        x,y = self.get_coordinates(pts, self.s, data)
        xn,yn = self.compress_important_points(x,y)
        m,b = self.get_lin_reg(xn,yn)
        xlin,ylin = self.get_line_data(m,b,xn,yn)

        VisualizationTools.plot_line(xlin, ylin, self.line_pub, frame="/laser")

    def get_important_points(self,scan_data):
        #scan_data
        ranges = np.array(scan_data.ranges)
        len_ranges = len(ranges)
        tri = len(ranges)//3
        mid_scan = ranges[tri:(2*tri)]
        sdp = []
        self.s = 'r'

        #obtain LHS values
        if self.SIDE == 1: #FLIPPING SIGN MAKES IT WORK
            left_scan = ranges[:tri]
            self.s = 'l'
            #check the values in the front of the car -- this will change what our slice consists of -- adds more points to slice
            #if object detected within a certain distance in front of the car, begin to include these values in the slice
            if np.mean(mid_scan) <= self.DESIRED_DISTANCE + 0.5:
                sdp = left_scan + mid_scan
            sdp = left_scan
        # #obtain RHS values
        else:
            right_scan = ranges[len_ranges-tri:]
            if np.mean(mid_scan) <= self.DESIRED_DISTANCE + 0.5:
                self.s = 'm'
                sdp = mid_scan + right_scan
            sdp = right_scan

        return sdp

    def get_coordinates(self, sd,side, scan_data):
        '''gets coordinates for linear regression line given scan data'''
        x_pts = []
        y_pts = []
        theta_increment = scan_data.angle_increment
        min_angle = scan_data.angle_min
        ranges = np.array(scan_data.ranges)
        tri = len(ranges)//3

        if side == 'l':
            initial = min_angle #LHS points start at index 0
        elif side == 'm':
            initial = min_angle + (tri*theta_increment) #Middle points start at index n/3
        else:
            initial = min_angle + (2*tri*theta_increment) #RHS points start at index 2n/3


        for x in range(len(sd)):
            theta = initial + (x*theta_increment)
            x_pts.append(sd[x]*np.cos(theta)) #convert to cartesian = rcostheta #CHECK THESE 
            y_pts.append(sd[x]*np.sin(theta))

        return np.array(x_pts),np.array(y_pts)

    def compress_important_points(self,x_pts,y_pts):
        '''obtains a smaller array of coordinate points to base lin_regression on'''
        idx = np.round(np.linspace(0, len(x_pts) - 1, 10)).astype(int) #obtain 10 evenly spaced elements from array
        return x_pts[idx], y_pts[idx]

    def get_lin_reg(self,x,y):
        '''obtains linear regression line'''
        return np.polyfit(x,y,1)

    def get_line_data(self,m,b,x,y):
        y_arr = []
        x_arr = x
        for i in x:
            y = m*i + b
            y_arr.append(y)

        return x_arr,y_arr


if __name__ == "__main__":
    rospy.init_node("line_publisher")
    line_publisher = LinePublisher()
    rospy.spin()
