#!/usr/bin/env python2

import csv
import numpy as np
import rospy
from std_msgs.msg import Float32 
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    
    def __init__(self):
        # Initialize Subscribers and Publishers
        self.subscriber = rospy.Subscriber(WallFollower.SCAN_TOPIC, LaserScan, self.callback)
        self.drive_publisher = rospy.Publisher(WallFollower.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.marker_publisher = rospy.Publisher("visualization_marker", Marker, queue_size = 10)
        self.error_publisher = rospy.Publisher("error", Float32, queue_size=10)
        self.lower_side = 3/23.
        self.upper_side = 11/23.
        self.lower_front = 11/23.
        self.upper_front = 13/23.
        self.max_delta = 0.34
        self.L = 0.343
        self.smallest_R = self.L/np.tan(self.max_delta)
        self.ku = 3.5
        self.T = 7./6.
        self.kp = self.ku*0.4
        self.kd = self.ku*0.3*self.T

    def callback(self, LaserScan_data):
        '''
        Expecting LaserScan data. Will use this function to publish steering commands
        '''
        # Initialize the drive data structure
        drive_data = self.initialize_drive_data()
        
        # Pull relevant data from LaserScan object
        ranges = np.array(LaserScan_data.ranges) 
        angle_min = LaserScan_data.angle_min
        angle_max = LaserScan_data.angle_max
        angle_difference = angle_max - angle_min
        angle_increment = LaserScan_data.angle_increment
        
        # Slice the data to relevant side
        if self.SIDE == 1:
            sliced_ranges = ranges[int(round(ranges.shape[0]*(1-self.upper_side))):int(round(ranges.shape[0]*(1-self.lower_side)))]
            scaling_array = np.arange((angle_difference)*(1-self.upper_side) + angle_min, (angle_difference)*(1-self.lower_side) + angle_min, angle_increment)
        else:
            sliced_ranges = ranges[int(round(ranges.shape[0]*self.lower_side)):int(round(ranges.shape[0]*self.upper_side))]
            scaling_array = np.arange((angle_difference)*self.lower_side + angle_min, (angle_difference)*self.upper_side + angle_min, angle_increment)

        # Slice the data to get front data
        front_ranges = ranges[int(round(ranges.shape[0]*self.lower_front)):int(round(ranges.shape[0]*self.upper_front))]
        front_scaling = np.arange((angle_difference)*self.lower_front + angle_min, (angle_difference)*self.upper_front + angle_min, angle_increment)

        # Convert side data into clean x, y from r, theta
        x_side, y_side = self.get_clean_cartesian_data(sliced_ranges, scaling_array, self.DESIRED_DISTANCE*1.5)

        # Convert front data into clean x, y from r, theta
        x_front, y_front = self.get_clean_cartesian_data(front_ranges, front_scaling, self.DESIRED_DISTANCE*1.5)
        
        # Regress the data points to estimate the side wall
        m_side, b_side = self.get_weighted_regression(x_side, y_side)
        
        # Regress the data points to estimate the front wall
        m_front, b_front = self.get_weighted_regression(x_front, y_front)

        # Create adjusted y value for first and last x value, side
        new_y_side = m_side*x_side + b_side
        
        # Get regressed min r, theta
        new_rs = np.sqrt(np.square(x_side) + np.square(new_y_side))
        min_r = np.amin(new_rs)
        min_angle = scaling_array[np.where(new_rs==min_r)]
        min_x = min_r*np.cos(min_angle)
        min_y = min_r*np.sin(min_angle)

        # Create adjusted y value for first and last x value, front
        new_y_front = m_front*x_front + b_front
        
        mid_inx_side = x_side.shape[0]/2
        mid_inx_front = x_front.shape[0]/2
        
        # Create points for each x, new_y pair, add to line Marker for side
        line_side = self.make_marker(Marker.LINE_STRIP, "side_line", (0.1, 0.1), (1.0, 1.0, 255, 1.0), [(x_side[0], new_y_side[0], 0.1), (x_side[mid_inx_side], new_y_side[mid_inx_side], 0.1), (x_side[-1], new_y_side[-1], 0.1)])
        
        # Create points for each x, new_y pair, add to line Marker for front
        line_front = self.make_marker(Marker.LINE_STRIP, "front_line", (0.1, 0.1), (1.0, 255, 0.0, 1.0), [(x_front[0], new_y_front[0], 0.1), (x_front[mid_inx_front], new_y_front[mid_inx_front], 0.1), (x_front[-1], new_y_front[-1], 0.1)])
        
        # Create marker for point with minimum distance
        point_mark = self.make_marker(Marker.POINTS, "closest_point", (0.2, 0.2), (0.0, 0.0, 255, 1.0), [(min_x, min_y, 0.1)])

        # Check how far we are from front wall:
        if self.get_dist_from_point(np.array([x_front[mid_inx_front], new_y_front[mid_inx_front]])) <= self.DESIRED_DISTANCE + self.smallest_R:
            # if within threshold, hard turn logic
            steering_angle = self.get_steering_angle_turn()
        else:
            # Drive straight logic, get steering angle
            steering_angle = self.get_steering_angle_straight(np.array([min_x, min_y]))
        drive_data.drive.steering_angle = steering_angle

        # PUBLISH THE CALCULATED STEERING ANGLE
        wall_follower.drive_publisher.publish(drive_data)
        wall_follower.marker_publisher.publish(line_side)
        wall_follower.marker_publisher.publish(line_front)
        wall_follower.marker_publisher.publish(point_mark)

    def initialize_drive_data(self):
        drive_data = AckermannDriveStamped()
        drive_data.drive.speed = self.VELOCITY
        drive_data.drive.acceleration = 0.5
        drive_data.drive.jerk = 0.0
        drive_data.drive.steering_angle = 0.0
        drive_data.drive.steering_angle_velocity = 0.5
        return drive_data

    def get_clean_cartesian_data(self, sliced_data, scaling_array, threshold):
        indexes = np.where(sliced_data >= threshold)
        np.delete(sliced_data, indexes)
        np.delete(scaling_array, indexes)

        x = sliced_data*np.cos(scaling_array)
        y = sliced_data*np.sin(scaling_array)
        return x, y
    
    def get_weighted_regression(self, x, y):
        r = np.sqrt(np.square(x) + np.square(y))
        weights = 1/r**3
        return np.polyfit(x, y, 1, w=weights)
    
    def make_marker(self, marker_type, ns, scale, color, points):
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.header.stamp = rospy.Time.now()
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.ns = ns
        marker.scale.x, marker.scale.y = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = (0, 0, 0, 1)
        for point in points:
            new_point = Point()
            new_point.x, new_point.y, new_point.z = point
            marker.points.append(new_point)
        return  marker

    def get_dist_from_point(self, point):
        return np.linalg.norm(point)
    
    def get_steering_angle_straight(self, min_point):
        # Get the relative distance from the car to the middle of the line
        rel_dist = self.get_dist_from_point(min_point)
        # caluclate the error term
        error = rel_dist - self.DESIRED_DISTANCE
        self.error_publisher.publish(error)
        theta = np.arctan2(min_point[1], min_point[0])
        
        if self.SIDE == 1:
            return self.SIDE*(error)*self.kp -self.SIDE*self.kd*np.cos(theta)*self.VELOCITY
        return self.SIDE*(error)*self.kp + self.SIDE*self.kd*np.cos(theta)*self.VELOCITY

    def get_steering_angle_turn(self):
        return -self.max_delta*self.SIDE
        
if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
