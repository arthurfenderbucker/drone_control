#!/usr/bin/env python
import rospy
import numpy as np
import math
# import mavros_msgs

from geometry_msgs.msg import Twist
# from mavros_msgs import srv
# from mavros_msgs.msg import State
from drone_control.msg import Vector3D
import time
#=================Parameter Initializaiton========================,
set_velocity = Twist()
# current_state = State()
current_state = None
control_mode = "position"


def takeoff():
    global set_velocity
    set_velocity.linear.z = 1
    set_velocity.linear.y = 0
    set_velocity.linear.x = 0
    setpoint_velocity_pub.publish(set_velocity)
    time.sleep(0.4)
    set_velocity.linear.z = 0
    set_velocity.linear.y = 0
    set_velocity.linear.x = 0
    setpoint_velocity_pub.publish(set_velocity)


def velocity_callback(goal_vec):
    global set_velocity
    set_velocity.linear.z = goal_vec.z
    set_velocity.linear.y = goal_vec.y
    set_velocity.linear.x = goal_vec.x

    global control_mode
    control_mode = "velocity"
    rospy.loginfo("got velocity goal")
    rospy.loginfo("x: " + str(goal_vec.x) + " y: " +
                  str(goal_vec.y) + " z: " + str(goal_vec.z))


#============Intialize Node, Publishers and Subscribers=================
rospy.init_node('Vel_Control_Node', anonymous=True)
rate = rospy.Rate(20)  # publish at 20Hz
#                           bebop
setpoint_velocity_pub = rospy.Publisher(
    '/bebop/cmd_vel', Twist, queue_size=10)
#takeoff()
set_velocity.linear.z = 1
set_velocity.linear.y = 0
set_velocity.linear.x = 0
while not rospy.is_shutdown():
    setpoint_velocity_pub.publish(set_velocity)
    
    rate.sleep()
rospy.loginfo("Shutting down")
