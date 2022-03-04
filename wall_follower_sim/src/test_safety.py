#!/usr/bin/env python2
import rospy
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped

class TestSafety:
    WALL_TOPIC = "/wall"
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic") #lidar sensor information, ranges = lidar to nearest obstacle
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side") #left is +1 , right is -1
    print(SIDE)
    VELOCITY = rospy.get_param("wall_follower/velocity")
    print(VELOCITY)
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    
    def __init__(self):
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size = 1)
        
    def talker(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            drive_data = self.drive_car()
            self.pub.publish(drive_data)

    def drive_car(self):
        ''' 
            commands car to brake 
            returns: drive_data (AckermannDriveStamped)
        '''
        drive_data = AckermannDriveStamped()
        drive_data.drive.acceleration = 0
        drive_data.drive.jerk = 0
        drive_data.drive.speed = self.VELOCITY 
        drive_data.drive.steering_angle = 0
        drive_data.drive.steering_angle_velocity = 0
        return drive_data


if __name__ == '__main__':
    try:
        rospy.init_node('test_safety')
        test_safety = TestSafety()
        test_safety.talker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
