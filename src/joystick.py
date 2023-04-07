#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped

from std_msgs.msg import String

import time

##TEM ALGUMA COISA CAGADEX AQUI #####

###  VERIFICAR COMO IMPORTAR A MENSAGEM DO PACKAGE ###

from drone_control.msg import Vector3D


def publicaVelocity(steps):  # list of x,y,z positions
    pub = rospy.Publisher('controle/velocity', Vector3D, queue_size=10)
    rospy.init_node('joystick', anonymous=True)
    rate = rospy.Rate(10)  # 0.5hz
    vec = Vector3D()

    step_index = 0
    while not rospy.is_shutdown():
        vec.x = steps[step_index[0]]
        vec.y = steps[step_index[1]]
        vec.z = steps[step_index[2]]
        for i in range(100):
            rospy.loginfo(vec)
            pub.publish(vec)
            rate.sleep()

        step_index += 1
        if step_index == len(steps):
            break


def publicaPosition(steps):

    pub = rospy.Publisher('controle/position', Vector3D, queue_size=10)

    rospy.init_node('joystick', anonymous=True)
    rate = rospy.Rate(10)  # 0.5hz
    vec = Vector3D()
    step_index = 0
    while not rospy.is_shutdown():

        vec.x = steps[step_index][0]
        vec.y = steps[step_index][1]
        vec.z = steps[step_index][2]
        for i in range(80):
            rospy.loginfo(vec)
            pub.publish(vec)
            rate.sleep()

        step_index += 1
        if step_index == len(steps):
            break


def manualPosition():
    pub = rospy.Publisher('controle/position', Vector3D, queue_size=10)

    rospy.init_node('joystick', anonymous=True)
    #   rate = rospy.Rate(10)
    vec = Vector3D()
    while not rospy.is_shutdown():
        pos_list = input("new position: x,y,z(ex: 1,0,2): ")
        vec.x = float(pos_list[0])
        vec.y = float(pos_list[1])
        vec.z = float(pos_list[2])

        rospy.loginfo(vec)
        pub.publish(vec)


def manualVelocity():
    pub = rospy.Publisher('controle/velocity', Vector3D, queue_size=10)

    rospy.init_node('joystick', anonymous=True)
    #   rate = rospy.Rate(10)
    vec = Vector3D()
    while not rospy.is_shutdown():
        pos_list = input("new velocity: x,y,z(ex: 1,0,2): ")
        vec.x = float(pos_list[0])
        vec.y = float(pos_list[1])
        vec.z = float(pos_list[2])

        rospy.loginfo(vec)
        pub.publish(vec)

# def pegaPose(msg):
#
#    posX = msg.pose.position.x
#    posY = msg.pose.position.y
#    posZ = msg.pose.position.z
#    print(1)

# def subscriber():

#    msg = PoseStamped()
#    rospy.Subscriber("controle/position", PoseStamped, pegaPose(msg))
#    rospy.spin()


if __name__ == '__main__':

    try:

        #publicaVelocity(10, 0, 0)
        publicaPosition([[0.0, 0.2, 1.3]])
        manualPosition()
        # manualVelocity()

        #steps = [[0.0, -0.2, 1.3], [2.8, 0.4, 1.3], [2.8, -2.5, 1.3]]
        # publicaPosition(steps)
    except rospy.ROSInterruptException or KeyboardInterrupt:
        print("exit")
        pass
