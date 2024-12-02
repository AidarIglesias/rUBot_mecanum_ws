#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class WallFollower:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('wall_follower')

        # Create a publisher to the /cmd_vel topic
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Create a subscriber to the /scan topic
        self.sub = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        # Register the shutdown function to be called on node shutdown
        rospy.on_shutdown(self.shutdown)

        # Set the loop rate to 5 Hz
        self.rate = rospy.Rate(20)

        # Get parameters from the parameter server
        self.d = rospy.get_param("~distance_laser")
        self.vx = rospy.get_param("~forward_speed")
        self.vy = rospy.get_param("~forward_speed")
        self.wz = rospy.get_param("~rotation_speed")
        self.vf = rospy.get_param("~speed_factor")

        # Variables for scan range correction factor
        self.isScanRangesLengthCorrectionFactorCalculated = False
        self.scanRangesLengthCorrectionFactor = 2

    def clbk_laser(self, msg):
        # On the first execution, calculate the correction factor
        if not self.isScanRangesLengthCorrectionFactorCalculated:
            self.scanRangesLengthCorrectionFactor = len(msg.ranges) / 360
            self.isScanRangesLengthCorrectionFactorCalculated = True

        # Define the range indices for different regions
        bright_min = int(30 * self.scanRangesLengthCorrectionFactor)
        bright_max = int(90 * self.scanRangesLengthCorrectionFactor)
        right_min = int(90 * self.scanRangesLengthCorrectionFactor)
        right_max = int(120 * self.scanRangesLengthCorrectionFactor)
        fright_min = int(120 * self.scanRangesLengthCorrectionFactor)
        fright_max = int(170 * self.scanRangesLengthCorrectionFactor)
        front_min = int(170 * self.scanRangesLengthCorrectionFactor)
        front_max = int(190 * self.scanRangesLengthCorrectionFactor)
        bleft_min = int(280 * self.scanRangesLengthCorrectionFactor)
        bleft_max = int(330 * self.scanRangesLengthCorrectionFactor)
        left_min = int(250 * self.scanRangesLengthCorrectionFactor)
        left_max = int(280 * self.scanRangesLengthCorrectionFactor)
        fleft_min = int(190 * self.scanRangesLengthCorrectionFactor)
        fleft_max = int(250 * self.scanRangesLengthCorrectionFactor)
        back_min = int(330 * self.scanRangesLengthCorrectionFactor)
        back_max = int(360 * self.scanRangesLengthCorrectionFactor)

        # Define the regions with minimum distances
        regions = {
            'bright':  min(min(msg.ranges[bright_min:bright_max]), 3),
            'right':  min(min(msg.ranges[right_min:right_max]), 3),
            'fright': min(min(msg.ranges[fright_min:fright_max]), 3),
            'front':  min(min(msg.ranges[front_min:front_max]), 3),
            'bleft': min(min(msg.ranges[bleft_min:bleft_max]), 3),
            'left': min(min(msg.ranges[left_min:left_max]), 3),
            'fleft': min(min(msg.ranges[fleft_min:fleft_max]), 3),
            'back': min(min(msg.ranges[back_min:back_max]), 3)
        }

        # Call the function to take appropriate action based on the regions
        self.take_action(regions)

    def take_action(self, regions):
        # Initialize the Twist message
        msg = Twist()
        linear_x = 0
        linear_y = 0
        angular_z = 0

        state_description = ''

        # Determine the state and actions based on the distances in the regions
        # if regions['front'] > self.d and regions['fright'] > 2*self.d and regions['right'] > 2*self.d and regions['bright'] > 2*self.d:
        #     state_description = 'case 0 - nothing'
        #     linear_x = self.vx
        #     angular_z = 0

        if regions['right'] < self.d:
            state_description = 'case 1 - right'
            linear_x = self.vx
            linear_y = 0
            angular_z = 0
        elif regions['front'] < self.d and regions['right'] < self.d:
            state_description = 'case 2 - front and right'
            linear_x = 0
            linear_y = self.vy
            angular_z = 0
        elif regions['front'] < self.d:
            state_description = 'case 3 - front'
            linear_x = 0
            linear_y = self.vy
            angular_z = 0
        elif regions['front'] < self.d and regions['left'] < self.d:
            state_description = 'case 4 - front and left'
            linear_x = -self.vx
            linear_y = 0
            angular_z = 0
        elif regions['left'] < self.d:
            state_description = 'case 5 - left'
            linear_x = -self.vx
            linear_y = 0
            angular_z = 0
        elif regions['fleft'] < self.d:
            state_description = 'case 6 - fleft'
            linear_x = 0
            linear_y = self.vy
            angular_z = 0
        elif regions['fright'] < self.d:
            state_description = 'case 7 - fright'
            linear_x = self.vx
            linear_y = 0
            angular_z = 0
        elif regions['back'] < self.d and regions['left'] < self.d:
            state_description = 'case 8 - back and left'
            linear_x = 0
            linear_y = -self.vy
            angular_z = 0
        elif regions['back'] < self.d:
            state_description = 'case 9 - back'
            linear_x = 0
            linear_y = -self.vy
            angular_z = 0
        elif regions['bleft'] < self.d:
            state_description = 'case 10 - bleft'
            linear_x = -self.vx
            linear_y = 0
            angular_z = 0
        elif regions['back'] < self.d and regions['right']:
            state_description = 'case 11 - back  and right'
            linear_x = self.vx
            linear_y = 0
            angular_z = 0
        else:
            state_description = 'case 5 - far'
            linear_x = 0
            linear_y = -self.vy
            angular_z = 0

        # if regions['front'] > self.d and regions['fright'] > 2*self.d and regions['right'] > 2*self.d and regions['bright'] > 2*self.d and regions['fleft'] > 2*self.d and regions['left'] > 2*self.d and regions['bleft'] > 2*self.d:
        #     state_description = 'case 1 - nothing'
        #     linear_x = self.vx
        #     angular_z = 0

        # elif regions['front'] < self.d and regions['right'] < self.d:
        #     state_description = 'case 2 - front and right'
        #     linear_x = 0
        #     linear_y = self.vy / 2
        #     angular_z = 0
        # elif regions['front'] < self.d and regions['left'] < self.d:
        #     state_description = 'case 3 - front and left'
        #     linear_x = 0
        #     linear_y = 0
        #     angular_z = self.wz
        # # elif regions['back'] < self.d and regions['right'] < self.d:
        # #     state_description = 'case 4 - back and right'
        # #     linear_x = self.vx
        # #     linear_y = 0
        # #     angular_z = 0
        # # elif regions['back'] < self.d and regions['left'] < self.d:
        # #     state_description = 'case 5 - back and left'
        # #     linear_x = 0
        # #     linear_y = -self.vy / 2
        # #     angular_z = 0

        # elif regions['front'] < self.d and regions['right'] > 2*self.d:
        #     state_description = 'case 6 - front and far right'
        #     linear_x = 0
        #     linear_y = self.vy / 2
        #     angular_z = 0
        # # elif regions['back'] < self.d:
        # #     state_description = 'case 7 - back'
        # #     linear_x = 0
        # #     linear_y = -self.vy
        # #     angular_z = 0
        # elif regions['front'] > 2*self.d and regions['right'] < self.d:
        #     state_description = 'case 8 - right and far front'
        #     linear_x = self.vx
        #     linear_y = 0
        #     angular_z = 0
        # # elif regions['left'] < self.d:
        # #     state_description = 'case 9 - left'
        # #     linear_x = -self.vx
        # #     linear_y = 0
        # #     angular_z = 0

        # elif regions['bright'] < self.d:
        #     state_description = 'case 10 - bright'
        #     linear_x = 0
        #     linear_y = -self.vy
        #     angular_z = 0
        # # elif regions['bleft'] < self.d:
        # #     state_description = 'case 11 - bleft'
        # #     linear_x = -self.vx
        # #     linear_y = 0
        # #     angular_z = 0
        # # elif regions['fright'] < self.d:
        # #     state_description = 'case 12 - fright'
        # #     linear_x = self.vx
        # #     linear_y = 0
        # #     angular_z = 0
        # # elif regions['fleft'] < self.d:
        # #     state_description = 'case 13 - fleft'
        # #     linear_x = 0
        # #     linear_y = self.vy / 2
        # #     angular_z = 0

        # else:
        #     state_description = 'case 14 - far'
        #     linear_x = self.vx
        #     angular_z = 0


        # Log the state description
        rospy.loginfo(state_description)
        # Set the linear and angular velocities in the Twist message
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        # Publish the Twist message to the /cmd_vel topic
        self.pub.publish(msg)
        # Sleep for the remaining time to maintain the loop rate
        self.rate.sleep()

    def shutdown(self):
        # Create a Twist message with zero velocities
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.angular.z = 0
        # Publish the Twist message to stop the robot
        self.pub.publish(msg)
        # Log the stop message
        rospy.loginfo("Stop")

if __name__ == '__main__':
    try:
        # Create an instance of the WallFollower class
        wall_follower = WallFollower()
        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        # Call the shutdown function if there is a ROS interrupt
        wall_follower.shutdown()