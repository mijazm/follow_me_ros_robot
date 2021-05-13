#!/usr/bin/env python3

# This code uses ROS to read joystick values and pubish control commands to a robot
# Author: Mijaz Mukundan

# References:
# 1. https://github.com/PacktPublishing/ROS-Robotics-By-Example
# 2. https://github.com/PacktPublishing/ROS-Robotics-By-Example/blob/master/Chapter_8_code/Turtlesim_joy_code/turtlesim_joy.py

# Arduino code for robot at https://github.com/mijazm/ROS_Arduino_PID

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

# WASD game based control, here the left joystick of the gamepad Logitech F710 is used
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# called when joy message is received
def tj_callback(data):

    # start publisher of cmd_vel to control Turtlesim
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # Create Twist message & add linear x and angular z from left joystick
    twist = Twist()
    
    #Use the X button to stop the robot
    if data.buttons[0] == 1:
        twist.linear.x = 0
        twist.angular.z = 0
    else:
        twist.linear.x = data.axes[1]
        twist.angular.z = data.axes[0]

    # record values to log file and screen
    # rospy.loginfo("twist.linear: %f ; angular %f", twist.linear.x, twist.angular.z)

    # publish cmd_vel
    pub.publish(twist)

def joy_listener():

    # start node
    rospy.init_node("control_with_joy", anonymous=True)

    # subscribe to joystick messages on topic "joy"
    rospy.Subscriber("joy", Joy, tj_callback, queue_size=1)

    # keep node alive until stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        joy_listener()
    except rospy.ROSInterruptException:
        pass
