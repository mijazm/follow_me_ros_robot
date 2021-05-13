#!/usr/bin/env python3

# This code generates PID Control by reading encoder ticks from motor and desired velocity 
# received from /cmd_vel topic
# Author: Mijaz Mukundan

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

#Parameters
wheeltrack = 0.35
wheelradius = 0.035
TPR = 1380 #Ticks per revolution from the SPG30E-200K DC Geared Motor with Encoder Datasheet


motors = [('left',0), ('right',0)]

#Ticks coming from the encoder of left and right motors
ticks = dict(motors) 

last_ticks = dict(motors)

#The required speed in RPM for left and right motors
motor_req_rpm = dict(motors) 

# Last error for PID calculation
last_error = dict(motors)

#Integral of error ( essentially sum for discrete system) for PID
int_error = dict(motors)


def leftTicksCallback(msg):
    global ticks 
    ticks['left'] = msg.data

def rightTicksCallback(msg):
    global ticks
    ticks['right'] = msg.data

def cmdVelCallback(msg):
    global motor_req_rpm

    linear_vel = msg.linear.x
    angular_vel = msg.angular.z

    if angular_vel == 0:
        motor_req_rpm['left'] = linear_vel*60 / (2*pi*wheelradius)
        motor_req_rpm['right'] = motor_req_rpm['left']
    
    elif linear_vel == 0:
        motor_req_rpm['right'] = (wheeltrack/2)*angular_vel*60 / (2*pi*wheelradius)
        motor_req_rpm['left'] = -motor_req_rpm['left']
    
    else:
        motor_req_rpm['left'] = (linear_vel-(wheeltrack/2)*angular_vel)*60/(2*pi*wheelradius)
        motor_req_rpm['right'] = (linear_vel+(wheeltrack/2)*angular_vel)*60/(2*pi*wheelradius)
    
    # print("LRPM:{},RRPM:{}".format(motor_req_rpm['left'],motor_req_rpm['right']))

def calc_pid(req_val,act_val,motor_id):
    Kp = 3
    Ki = 0
    Kd = 0
    global last_error, int_error
    
    #Calculate error
    error = req_val - act_val

    #Update integral error
    int_error[motor_id]= int_error[motor_id] + error

    #Error difference
    diff_error = error - last_error[motor_id]
    
    #Calculate PID
    pid_term = Kp*error + Ki*int_error[motor_id] + Kd*diff_error

    last_error[motor_id] = error

    return pid_term



rospy.init_node('pid_publisher')

#Subscribed to ..
cmd_vel_sub = rospy.Subscriber("/cmd_vel",Twist,cmdVelCallback)
left_ticks_sub =rospy.Subscriber("/lwheel", Int16, leftTicksCallback)
right_ticks_sub =rospy.Subscriber("/rwheel", Int16, rightTicksCallback)

#Publish to ..
pid_pub = rospy.Publisher("/pid_control", Twist, queue_size=1)


last_time = rospy.Time.now()

r = rospy.Rate(10)

while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    # Change in wheel position
    delta_L = ticks['left'] - last_ticks['left']
    delta_R = ticks['right'] - last_ticks['right']

    dt = (current_time - last_time).to_sec()
    
    #Actual Wheel RPM calculation
    left_motor_act_rpm  = delta_L / TPR * (60/dt)
    right_motor_act_rpm = delta_R / TPR * (60/dt)

    # print("ACT RPM:")
    # print("l:{},r:{}".format(left_motor_act_rpm,right_motor_act_rpm))
    # print("rl:{},rr:{}".format(motor_req_rpm['left'],motor_req_rpm['right']))
    
    #Motor PID term calculation
    left_pid = calc_pid(motor_req_rpm['left'],left_motor_act_rpm,motor_id='left')
    right_pid = calc_pid(motor_req_rpm['right'],right_motor_act_rpm,motor_id='right')
    
    #we will hijack Twist() message to send left and right motor pid values
    ctrl_msg = Twist() 
    ctrl_msg.linear.x = left_pid
    ctrl_msg.angular.z = right_pid
    
    #Publish the control message
    pid_pub.publish(ctrl_msg)

    
    last_ticks['left'] = ticks['left']
    last_ticks['right'] = ticks['right']
    last_time = current_time
    r.sleep()