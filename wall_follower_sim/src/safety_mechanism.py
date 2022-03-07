#!/usr/bin/env python2
import rospy
import math
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyMechanism:
    WALL_TOPIC = "/wall"
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic") #lidar sensor information, ranges = lidar to nearest obstacle
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side") #left is +1 , right is -1
    VELOCITY = rospy.get_param("wall_follower/velocity")
    print(VELOCITY)
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    
    STOPPING_DIST = VELOCITY*0.34 + 0.3 
    SAFETY_TOPIC = "/vesc/low_level/ackermann_cmd_mux/input/safety"
    DISTANCE_TOPIC = "distance_topic"
    DRIVE_OUT_TOPIC = "/vesc/high_level/ackermann_cmd_mux/output"

    drive_cmd = AckermannDriveStamped

    def __init__(self):
        #init safety publisher
        self.safety_pub = rospy.Publisher(self.SAFETY_TOPIC, AckermannDriveStamped, queue_size=1) #topic
                
        #initialize subscriber
        self.scan_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        self.drive_sub = rospy.Subscriber(self.DRIVE_OUT_TOPIC, AckermannDriveStamped, self.new_drive)

    def new_drive(self, drive):
        self.drive_cmd = drive

    def callback(self,scan_data):
        stop, min_dist = self.stopping_mechanism(scan_data)
        if stop:
            print("STOP")
            print(min_dist)
            
            #obtain car data and control car
            drive_data = self.stop_car()
            self.safety_pub.publish(drive_data)

    def stopping_mechanism(self,scan_data):
        ''' 
            given a range of angles, tells the car to stop if a certain number of data points are within a certain
            distance from the car 
            params: scan_data (LaserScan)
            returns: True if the car needs to stop / False if the car does not need to stop
        '''
        ranges = np.array(scan_data.ranges)
        angle_increment = scan_data.angle_increment
        min_angle = scan_data.angle_min
        max_angle = scan_data.angle_max

        #+- 15 degrees from straight ahead, 30 degrees total
        start = -15*np.pi/180
        end = 15*np.pi/180

        subset = self.get_subset(ranges,start,end,min_angle,angle_increment)
        subset_filtered = subset[np.where(subset > 0.05)]
        min_dist = np.amin(subset_filtered)
        #check if we need to stop given subset data -- if more than 10 percet of data points are within a distance
        if self.drive_cmd.drive.speed < 0:
            return False, min_dist
        return np.shape(np.where(subset<=self.STOPPING_DIST))[1] > len(subset)*.10, min_dist

    def get_subset(self,ranges,start,end,min_angle,angle_increment):
        ''' 
            gets a subset of range values 
            params: ranges array, start_angle, end_angle, min_angle, angle_increment
            returns: desired subset of Lidar range data []
        '''
        s = round((start-min_angle)/angle_increment)
        e = round((end-min_angle)/angle_increment)

        return ranges[int(s):int(e)]

    def stop_car(self):
        ''' 
            commands car to brake 
            returns: drive_data (AckermannDriveStamped)
        '''
        drive_data = AckermannDriveStamped()
        drive_data.drive.acceleration = 0
        drive_data.drive.jerk = 0
        drive_data.drive.speed = 0
        drive_data.drive.steering_angle = 0
        drive_data.drive.steering_angle_velocity = 0
        return drive_data


if __name__ == '__main__':
    try:
        rospy.init_node('safety_mechanism')
        safetyMechanism = SafetyMechanism()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
