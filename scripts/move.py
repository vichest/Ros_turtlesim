#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

def move_turtle():
    # Initialize the ROS node
    rospy.init_node('move_turtle_rectangle', anonymous=True)
    
    # Create a publisher to send velocity commands to the turtle
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # Define the rate at which the loop will run (10 Hz)
    rate = rospy.Rate(10)
    
    # Define linear and angular velocity values
    linear_speed = 1.0  # Adjust for speed along each side
    angular_speed = 0.5  # Adjust for slower, more precise turns

    # Define the rectangle dimensions (length and width in meters)
    length = 2.0  # Length of the rectangle
    width = 1.0   # Width of the rectangle

    # Calculate travel time based on speed
    time_length = length / linear_speed
    time_width = width / linear_speed
    turn_duration = 1.57 / angular_speed  # Approx time for a 90-degree turn (1.57 radians)

    vel_msg = Twist()

    # Function to move in a straight line for a given duration
    def move_straight(time_duration):
        vel_msg.linear.x = linear_speed
        vel_msg.angular.z = 0.0
        start_time = time.time()
        while time.time() - start_time < time_duration and not rospy.is_shutdown():
            velocity_publisher.publish(vel_msg)
            rate.sleep()
        # Stop after moving straight
        vel_msg.linear.x = 0.0
        velocity_publisher.publish(vel_msg)

    # Function to turn 90 degrees precisely
    def turn_90_degrees():
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = angular_speed
        start_time = time.time()
        while time.time() - start_time < turn_duration and not rospy.is_shutdown():
            velocity_publisher.publish(vel_msg)
            rate.sleep()
        # Stop turning after rotation
        vel_msg.angular.z = 0.0
        velocity_publisher.publish(vel_msg)

    rospy.loginfo("Moving the turtle in a rectangle pattern...")

    while not rospy.is_shutdown():
        # Move along the length of the rectangle
        move_straight(time_length)
        
        # Turn 90 degrees
        turn_90_degrees()
        
        # Move along the width of the rectangle
        move_straight(time_width)
        
        # Turn 90 degrees
        turn_90_degrees()

if __name__ == '__main__':
    try:
        # Call the function to move the turtle in a rectangle
        move_turtle()
    except rospy.ROSInterruptException:
        pass
