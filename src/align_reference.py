#!/usr/bin/env python
import rospy
import numpy as np
import math

from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Point, Vector3
from std_msgs.msg import Bool, Float32, Empty, String
from PID import PID

import json
import rospkg
rospack = rospkg.RosPack()

config_path= str(rospack.get_path('drone_control')+'/config/align_pid_config.json')


class adjust_position():
    image_shape = np.array([856,480])
    bool_align = False
    last_ref_point_time = 0
    
    goal_point = np.append( image_shape.copy() / 2, 100) # center of the image as default
    current_point = goal_point.copy() 
    precision = np.array([40,40,5]) # pixels

    count_aligned = 0

    # vertical angle between camera's perpenficular vector and horizontal plane
    camera_angle = 1.57
    
    tracking = False
    last_ref_point_time = 0.0

    # pid_x_calibration_data_p = [0.0014,0.0018,0.18,0.18] #value and distance
    # pid_x_calibration_data_i = [0.00002,0.0002,0.000004,0.000004] #value and distance
    # pid_x_calibration_data_d = [0.35,0.7,1.4,1.4] #value and distance

    speed = 0.5

    # pid_x_calibration_data_dist = [ 3 ,4, 5  ,30] # [ 2.16 ,3.52,4.36,30]

    # pid_x = PID(P=0.0005,I=0.0000001,D=0.002)
    # pid_y = PID(P=0.001, I=0.0000001,D=0.002)
    # pid_z = PID(P=0.0006,I=0.0000001,D=0.0006)

    pid_x = PID(P=0.0005,I=0.0000000,D=0.0005)
    pid_y = PID(P=0.0005, I=0.00000,D=0.0005)
    pid_z = PID(P=0.005,I=0.0000000,D=0.0001)

    pid_x.setPoint(goal_point[0])
    pid_y.setPoint(goal_point[1])
    pid_z.setPoint(goal_point[2])
    
    target_radium_real_size = 0.23
    f = 320 #equivalent focal length

    def __init__(self):

        rospy.init_node('align_reference', anonymous=True)
        self.rate = rospy.Rate(25) # 20hz

        self.running = rospy.get_param('~running',False)
        pid_config_name = rospy.get_param('~pid_config_name','default')

        self.update_pid_config(pid_config_name)


        rospy.Subscriber(
                    "/control/align_reference/ref_point", Point, self.point_callback, queue_size=1)
        rospy.Subscriber(
                    "/control/align_reference/set_image_shape", Point, self.set_image_shape, queue_size=1)
        rospy.Subscriber(
                    "/control/align_reference/set_goal_point", Point, self.set_goal_point, queue_size=1)
        rospy.Subscriber(
                    "/control/align_reference/set_speed", Float32, self.set_speed, queue_size=1)
        rospy.Subscriber(
                    "/control/align_reference/set_precision", Point, self.set_precision, queue_size=1)
        rospy.Subscriber(
                    "/control/align_reference/set_pid_config", String, self.set_pid_config, queue_size=1)
        rospy.Subscriber(
                    "/control/align_reference/set_camera_angle", Float32, self.set_camera_angle, queue_size=1)
        rospy.Subscriber('/bebop/land', Empty, self.land,queue_size=10)
        rospy.Subscriber('/bebop/reset', Empty, self.land,queue_size=10)
        self.running_sub= rospy.Subscriber(
            "control/align_reference/set_running_state", Bool, self.set_running_state, queue_size=1)
        
        self.setpoint_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self.aligned = rospy.Publisher(
            "/control/align_reference/aligned", Bool, queue_size=10)
        self.pid_error_pub = rospy.Publisher(
            "/control/align_reference/pid_error", Point, queue_size=1)

        

    # ================ topic callback functions ===================
    def set_pid_config(self, data):#String
        self.update_pid_config(data.data)
        
    def update_pid_config(self, pid_config_name): #loads config data from json file
        with open(config_path) as json_data_file:
            pid_config_file = json.load(json_data_file)
            if pid_config_name in pid_config_file.keys():
                pid_config_data = pid_config_file[pid_config_name]
            else:#default
                rospy.logerr("could not find the specified pid_config key, loading the default config!")
                pid_config_data = {"x":{"P":0.0005,"I":0.0000001, "D":0.002},"y":{"P":0.001,"I":0.0000001, "D":0.002},"z":{"P":0.0006,"I":0.0000001, "D":0.0006}}
        for k in pid_config_data.keys():
            print(str(k)+": P: " + str(pid_config_data[k]["P"])+" I: " + str(pid_config_data[k]["I"])+ " D: " + str(pid_config_data[k]["D"]))
        self.pid_x = PID(P=pid_config_data["x"]["P"],I=pid_config_data["x"]["I"],D=pid_config_data["x"]["D"])
        self.pid_y = PID(P=pid_config_data["y"]["P"],I=pid_config_data["y"]["I"],D=pid_config_data["y"]["D"])
        self.pid_z = PID(P=pid_config_data["z"]["P"],I=pid_config_data["z"]["I"],D=pid_config_data["z"]["D"])
        self.pid_x.setPoint(self.goal_point[0])
        self.pid_y.setPoint(self.goal_point[1])
        self.pid_z.setPoint(self.goal_point[2])

    def set_speed(self,data): #float
        self.speed = data.data
    def land(self,data):#empty
        self.running = False
        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        self.setpoint_vel_pub.publish(vel)

    def point_callback(self, data): #point
        self.last_ref_point_time = rospy.get_time()
        self.bool_align = True

        self.current_point = np.array([data.x,data.y,data.z])
        
    def set_image_shape(self,data): #point
        self.image_shape =  np.array([data.x,data.y])
        self.current_point = self.goal_point.copy()
    def set_precision(self, data): #point
        self.precision =  np.array([data.x,data.y,data.z])

    def set_goal_point(self,data): #point
        self.goal_point = np.array([data.x,data.y,data.z])
        self.pid_x.setPoint(data.x)
        self.pid_y.setPoint(data.y)
        self.pid_z.setPoint(data.z)
        self.bool_align = False
        self.count_aligned = 0

    def set_running_state(self,boolean_state):
        self.running = boolean_state.data
        self.reset_pid()

    def set_camera_angle(self,data):#float
        print(data.data)
        self.camera_angle = data.data
    # ================== control functions ========================
    def pub_vel(self,vel):
        setted_vel = Twist()

        # note: add angle correction
        setted_vel.linear.x = vel[2] *self.speed
        setted_vel.linear.y = vel[0] *self.speed
        setted_vel.linear.z = vel[1] *self.speed
        print(setted_vel)
        self.setpoint_vel_pub.publish(setted_vel)


    def run(self):
        while not rospy.is_shutdown():
            if self.running:
                #check_reference_lost (no references received in 1 second)
                if rospy.get_time() - self.last_ref_point_time > 0.3 :
                    
                    #stop drone
                    vel = np.array([0,0,0])
                    self.bool_align = False
                    self.count_aligned = 0
                    self.tracking = False
                    rospy.loginfo("reference lost")
                else:
                    vel = self.calculate_vel()
                    self.tracking = True

                self.pub_vel(vel)
                print(self.camera_angle)
            self.rate.sleep()

    def reset_pid(self):
        self.pid_x.setIntegrator(0)
        self.pid_y.setIntegrator(0)
        self.pid_z.setIntegrator(0)
    def get_distance(self):
        d = self.f*self.target_radium_real_size/self.current_point[2]
        print(d)
        return d
    def calculate_vel(self):
        print(self.get_distance())

        
        vel_raw = np.array([0.0,0.0,0.0])

        vel_raw[0] = self.pid_x.update(self.current_point[0])
        vel_raw[1] = self.pid_y.update(self.current_point[1])
        vel_raw[2] = self.pid_z.update(self.current_point[2])
        
        # if abs(self.pid_x.getError()) > self.precision[0] or abs(self.pid_y.getError()) > self.precision[1]:
        #     vel_raw[2] = 0
        # vel_raw[2] = 0
        cos = np.cos(self.camera_angle)
        sin = np.sin(self.camera_angle)

        camera_tf = np.array([[ 1.0,0.0,0.0],
                              [ 0.0,cos,-sin],
                              [ 0.0,sin,cos]])
        # vel = vel_raw
        vel = np.dot(camera_tf,vel_raw)
        e = Point()
        e.x = self.pid_x.getError()
        e.y = self.pid_y.getError()
        e.z = self.pid_z.getError()
        print(e.x,e.y,e.z)
        self.pid_error_pub.publish(e)
        # print(vel)
        if abs(e.x) > self.precision[0] or abs(e.y) > self.precision[1] or abs(e.z) > self.precision[2]:
            self.bool_align = False
            self.count_aligned = 0
        elif self.tracking:
            self.count_aligned += 1

        if self.count_aligned > 3:
            rospy.loginfo("ALIGNED!!")
            
            self.bool_align = True
            self.aligned.publish(True)
        
        return vel

    
def main():

    aligner =  adjust_position()
    aligner.run()

if __name__ == "__main__":
    main()
