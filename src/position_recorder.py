#!/usr/bin/env python

import rospy
import json
from geometry_msgs.msg import Pose
import ros_numpy
import rospkg


rospack = rospkg.RosPack()
positions_path = str(rospack.get_path('drone_control')+'/config/recorded_routines.json')


with open(positions_path, 'r') as json_data_file:
    try:
        positions_data = json.load(json_data_file)
    except:
        positions_data = {}

decision = 0
pose = []

def pose_callback(data):
    global pose
    pose = ros_numpy.numpify(data).tolist()
    
rospy.init_node('position_recorder', anonymous=True)
sub = rospy.Subscriber("/odom_slam_sf/current_pose", Pose, pose_callback, queue_size=1)

routine_name = "default"
while decision != 2 :
    print("Selected routine: "+routine_name)
    print (' 1-5: save position \n  7: change routine \n 8: remove routine \n 9: exit ')
    decision = input()                                                                                                         
    if decision >= 1 and decision < 7:
        if len(pose)>0: 
            if routine_name in positions_data.keys():
                positions_data[routine_name] = positions_data[routine_name]+[pose]
            else:
                positions_data[routine_name] = [pose]

            with open(positions_path, 'w') as json_data_file:
                json.dump(positions_data, json_data_file)
            print("position saved")
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
        routine_name = str(raw_input("please write the name of the routine edit or create: "))
        if routine_name in positions_data.keys():
            print("existing routine selected!")    
            print(positions_data[routine_name])
        else:
            print("new routine selected!")
    if decision == 8: #delete routine
        if str(raw_input("are you sure? (y/n):")) == "y":
            positions_data.pop(routine_name,None)
            routine_name = "default"
            print("routine deleted")
    print("\n\n\n")
        