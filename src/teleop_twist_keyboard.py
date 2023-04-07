#!/usr/bin/env python

from __future__ import print_function

# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist, Point, Pose
from std_msgs.msg import String, Empty, Bool

import sys, select, termios, tty

import json
from geometry_msgs.msg import Pose
import ros_numpy
import rospkg
import os, sys
sys.path.append(os.path.dirname(__file__)+"/../../imav_indoor/scripts/")
from sm_states import *


rospack = rospkg.RosPack()
positions_path = str(rospack.get_path('drone_control')+'/config/recorded_routines.json')


with open(positions_path, 'r') as json_data_file:
    try:
        positions_data = json.load(json_data_file)
    except:
        positions_data = {}


msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

add current position to routine:
    1:shelf    2:qr1    3:qr2    4:box    5:drop_box
    *: delete all or current routines


Actions:
    '=' : land over box  
    '{':take_off              '}': land
    'A':'toogle_inventory',
    's':'stop_reading_qr_code',
    'd':'read_flag'

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }
record_position  ={
    '1':'shelf',
    '2':'qr1',
    '3':'frame1',
    '4':'qr2',
    '5':'frame2',
    '6':'box',
    '7':'deliver_box',
    '*':'delete last'}
    
actions = {
    '{':'take_off',
    '}':'land',
    '=':'face_box',
    'A':'toogle_inventory',
    's':'stop_reading_qr_code',
    'd':'read_flag'}


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

pose = []

def pose_callback(data):
    global pose
    pose = ros_numpy.numpify(data).tolist()

face_box_state = face_box()

drop_box_state = drop_box()
pickup_box_state = pickup_box()
inventory_state = inventory()


sub = rospy.Subscriber("/odom_slam_sf/current_pose", Pose, pose_callback, queue_size=1)
takeoff_topic = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
# takeoff_topic = rospy.Publisher("/tello/takeoff", Empty, queue_size=1)

land_topic = rospy.Publisher("/bebop/land", Empty, queue_size=1)
# land_topic = rospy.Publisher("/tello/land", Empty, queue_size=1)

read_tag_pub= rospy.Publisher("cv_detection/inventory/read_tag", Empty, queue_size=1)
running_inventory_pub= rospy.Publisher("cv_detection/inventory/set_runnig_state", Bool, queue_size=1)
stop_reading_qr_pub= rospy.Publisher("cv_detection/inventory/stop_reading_qr", Empty, queue_size=1)

running_inventory = True



if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
    # pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size = 1)

    rospy.init_node('teleop_twist_keyboard')
    running_inventory_pub.publish(running_inventory)

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    # try:
    print(msg)
    print(vels(speed,turn))
    while(1):
        key = getKey()
        if key in moveBindings.keys():
            x = moveBindings[key][0]
            y = moveBindings[key][1]
            z = moveBindings[key][2]
            th = moveBindings[key][3]
        else:
            x = 0
            y = 0
            z = 0
            th = 0
            if (key == '\x03'):
                break
        if key in speedBindings.keys():
            speed = speed * speedBindings[key][0]
            turn = turn * speedBindings[key][1]

            print(vels(speed,turn))
            if (status == 14):
                print(msg)
            status = (status + 1) % 15
        if key in record_position.keys():

            if key == '*': #delete all routine
                c= str(input("DELETE routine(s)! are you sure? (a(all)/y/N):"))
                if c == "y":
                    positions_data.pop(routine_name,None)
                    routine_name = "default"
                    print("routine deleted")
                elif c == "a":
                    positions_data = {}
                    routine_name = "default"
                    print("all routine deleted")
            else:
                decision = int(key)
                # =========================== position recorder ====================================
                if decision >= 1 and decision <= 7:
                    if len(pose)>0:
                        routine_name = record_position[key] 

                        if routine_name in positions_data.keys():
                            positions_data[routine_name] = positions_data[routine_name]+[pose]
                        else:
                            positions_data[routine_name] = [pose]
                        with open(positions_path, 'w') as json_data_file:
                            json.dump(positions_data, json_data_file)
                        print("position saved")
                        print(routine_name+" contains: "+str(len(positions_data[routine_name])) + " positions")
                    else:
                        print("no new positions have been received yet!\nplease check if the \"control.py\" node is running and if the drone is connected")
                if decision == 6:
                    if len(pose)>0:
                        if routine_name in positions_data.keys():
                            positions_data[routine_name] = positions_data[routine_name][:-1]
                        else:
                            print("no routine found!")

                        with open(positions_path, 'w') as json_data_file:
                            json.dump(positions_data, json_data_file)
                        print("position saved")

                if decision == 7:
                    print("existing routines:")
                    print(positions_data.keys())
                    routine_name = str(input("please write the name of the routine edit or create: "))
                    if routine_name in positions_data.keys():
                        print("existing routine selected!")    
                        print(positions_data[routine_name])
                    else:
                        print("new routine selected!")
                if decision == 9: #delete all routine
                    pass

            with open(positions_path, 'w') as json_data_file: #save data
                json.dump(positions_data, json_data_file)
        
        elif key in actions.keys():
            if key == '{':
                takeoff_topic.publish(Empty())
            elif key == '}':
                land_topic.publish(Empty())
            elif key == '=':
                face_box_state.execute(None)
                pickup_box_state.execute(None)
                land_topic.publish(Empty())
            elif key == 'A':
                running_inventory_pub.publish(running_inventory)
                running_inventory= not running_inventory
            elif key == 's':
                stop_reading_qr_pub.publish(Empty())
            elif key == 'd':
                read_tag_pub.publish(Empty())
            
        print(key)
        twist = Twist()
        twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
        pub.publish(twist)

    # except Exception as e:
    #     print(e)

    # finally:
    #     twist = Twist()
    #     twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    #     twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    #     pub.publish(twist)

    #     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
