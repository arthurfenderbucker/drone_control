#!/usr/bin/env python

import rospy
import numpy as np


from geometry_msgs.msg import Point, Pose
from std_msgs.msg import String

def array_to_Pose(v):
    pose = Pose()
    pose.position.x = v[0]
    pose.position.y = v[1]
    pose.position.z = v[2]
    pose.orientation.w = 1
    return pose

def main():
    pub = rospy.Publisher('/control/position_relative', Pose, queue_size=1)
    rospy.init_node('zigzag', anonymous=True)
    rate =rospy.Rate(0.125)
    while not rospy.is_shutdown():
        
        rospy.loginfo("[0,0,0.5]")
        pub.publish(array_to_vec([0,0,0.5]))

        t = raw_input()
        rospy.loginfo("[0,0,-0.5]")
        pub.publish(array_to_vec([0,0,-0.5])
        t = raw_input()
        # rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

