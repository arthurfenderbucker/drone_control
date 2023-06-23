#!/usr/bin/env python
import rospy
import numpy as np
import math

from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Point, Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty, Bool
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse, SetBool, SetBoolRequest, SetBoolResponse
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
import ros_numpy

from PID import PID

from dynamic_reconfigure.server import Server
from drone_control.cfg import ControlConfig

import time
import tf
import yaml
import rospkg

rospack = rospkg.RosPack()

class vso_controler(object): # visual odometry drone controler
    
    goal_pose = Pose()
    current_pose = Pose()
    goal_pose.orientation.w = 1
    goal_pose.position.z = 1
    positioning_vel = np.array([0.0,0.0,0.0,0.0])

    


    camera_angle = Twist()
    setted_vel = Twist()

    control_mode = "position" # position or velocity  
    precision = np.array([0.15,0.15,0.1,0.1])
    count_aligned = 0
    def __init__(self):

        #setup node
        rospy.init_node('Vel_Control_Node', anonymous=True)
        self.rate = rospy.Rate(60)  # refresh rate (Hz)


        self.config_file = rospy.get_param('~config_file',str(rospack.get_path('drone_control')+'/config/calibration.yaml'))

        #load parameters from config file
        param, success = self.load_config()


        if success:
            self.pid_x = PID(P = param["P_x"]*10**param["mult_P_x"], I = param["I_x"]*10**param["mult_I_x"], D = param["D_x"]*10**param["mult_D_x"])
            self.pid_y = PID(P = param["P_y"]*10**param["mult_P_y"], I = param["I_y"]*10**param["mult_I_y"], D = param["D_y"]*10**param["mult_D_y"])
            self.pid_z = PID(P = param["P_z"]*10**param["mult_P_z"], I = param["I_z"]*10**param["mult_I_z"], D = param["D_z"]*10**param["mult_D_z"])
            self.pid_ang = PID(P = param["P_ang"]*10**param["mult_P_ang"], I = param["I_ang"]*10**param["mult_I_ang"], D= param["D_ang"]*10**param["mult_D_ang"])
            self.running = param["running"]
        else:
            self.pid_x = PID(P=0.4,I=0.0,D=0.8)
            self.pid_y = PID(P=0.4,I=0.0,D=0.8)
            self.pid_z = PID(P=0.3,I=0.00001,D=0.0012)
            self.pid_ang = PID(P=0.2,I=0.0,D=0.003)
            self.running = True


        self.aligned_xyz = False
        self.last_time = None
        self.last_goal_pose = Pose()
        self.last_goal_pose_time = rospy.Time.now()

        #topics and services
        self.setpoint_velocity_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
        self.setpoint_moveby_pub = rospy.Publisher('/bebop/moveby', Twist, queue_size=1)

        self.camera_angle_pub = rospy.Publisher('/bebop/camera_control', Twist, queue_size=10)

        self.running = rospy.get_param('~running',True)
        self.vso_on = rospy.get_param('~vso_on',True)
        self.max_speed = rospy.get_param('~max_speed',0.05)
        self.openloop_supervised = rospy.get_param('~openloop_supervised',False)


        self.config_file = rospy.get_param('~config_file',"default.json")

        calibrate_pid = rospy.get_param('~calibrate_pid',False)

        rospy.Subscriber('/bebop/land', Empty, self.land)
        rospy.Subscriber('/bebop/reset', Empty, self.land)
        # rospy.Subscriber('/bebop/takeoff', Empty, self.takeoff)
        
        rospy.Subscriber('/odom_slam_sf/current_pose', Pose, self.current_pose_callback)

        
        rospy.Subscriber('/control/position', Pose, self.position_callback)
        rospy.Subscriber('/control/position_relative', Pose, self.position_relative_callback)
        rospy.Subscriber('/control/velocity', Point, self.velocity_callback, queue_size=10)
        rospy.Subscriber('/control/set_precision', Point, self.set_precision)
        rospy.Subscriber('/control/pickup_box', Empty, self.pickup_box)
        
        rospy.Subscriber('/control/set_manual_mode', Bool, self.set_manual_mode_cb)


        # rospy.Subscriber('/control/land', Empty, self.land)

        rospy.Service('/control/calibrate_pid', SetBool, self.set_calibrate_pid)

        self.running_sub = rospy.Subscriber(
            "control/set_running_state", Bool, self.set_running_state, queue_size=1)
        
        self.current_pose_pub = rospy.Publisher(
            "control/current_position", Pose, queue_size=1)
        self.error_pose_pub = rospy.Publisher(
            "control/error", Pose, queue_size=1)
        self.aligned = rospy.Publisher(
            "/control/aligned", Bool, queue_size=1)
            
        #dynamic parameters serve
        if calibrate_pid:
            srv = Server(ControlConfig, self.parameters_callback)
        
        t = time.time()
        rospy.loginfo("aligning camera")
        while time.time() - t < 1:
            self.align_camera()
        rospy.loginfo("setup ok")
        self.pid_setpoint(self.goal_pose)
        # self.reset()

    def load_config(self):
        param = {}
        success = False
        with open(self.config_file, 'r') as stream:
            try:
                param = yaml.load(stream, Loader=yaml.Loader)
                rospy.loginfo("Control parameters loaded")
                success = True

            except yaml.YAMLError as exc:
                print(exc)
                rospy.loginfo("Error loading control parameters! Using default values")

        return param, success


    # -============================== topics callbacks ==============================
    def set_precision(self,data):
        self.precision = np.array([data.x,data.y,data.z,self.precision[3]])
    def set_running_state(self,boolean_state):
        self.running = boolean_state.data
        self.reset_pid()

    def land(self,callback_data):
        self.running = False
        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        self.setpoint_velocity_pub.publish(vel)
    
    def set_manual_mode_cb(self,msg):
        self.running = not msg.data
        print(self.running)
        vel_zero = Twist()
        vel_zero.linear.x = 0
        vel_zero.linear.y = 0
        vel_zero.linear.z = 0
        self.setpoint_velocity_pub.publish(vel_zero)

        

    def takeoff(self,callback_data):
        self.align_camera()
        
    def position_callback(self, goal_pose):
        self.goal_pose = goal_pose
        self.goal_pose_np = ros_numpy.numpify(goal_pose)
        self.goal_z_ang = self.euler_from_pose(self.goal_pose)[2]
        self.pid_setpoint(self.goal_pose)
        self.control_mode = "position"
        self.count_aligned = 0

    def position_relative_callback(self, relative_pose):

        ang_z = self.euler_from_pose(self.current_pose)[2]
        # print(ang_z)
        delta_x = relative_pose.position.x*np.cos(ang_z)-relative_pose.position.y*np.sin(ang_z)
        delta_y = relative_pose.position.y*np.cos(ang_z)+relative_pose.position.x*np.sin(ang_z)

        self.goal_pose.position.x += delta_x
        self.goal_pose.position.y += delta_y

        self.goal_pose.position.z += relative_pose.position.z
        new_z_ang = self.euler_from_pose(self.goal_pose)[2] + self.euler_from_pose(relative_pose)[2]
        if new_z_ang < -np.pi: new_z_ang+= np.pi
        if new_z_ang > np.pi: new_z_ang-= np.pi
        quarterion = tf.transformations.quaternion_from_euler(0,0,new_z_ang)
        self.goal_pose.orientation.x = quarterion[0]
        self.goal_pose.orientation.y = quarterion[1]
        self.goal_pose.orientation.z = quarterion[2]
        self.goal_pose.orientation.w = quarterion[3]
        self.pid_setpoint(self.goal_pose)
        # print(self.goal_pose)
        self.control_mode = "position"
        self.count_aligned = 0
    # def position_relative_callback(self, new_goal_pose):
    #     self.goal_pose.position.x += new_goal_pose.position.x
    #     self.goal_pose.position.y += new_goal_pose.position.y
    #     self.goal_pose.position.z += new_goal_pose.position.z
    #     new_z_ang = self.euler_from_pose(self.goal_pose)[2] + self.euler_from_pose(new_goal_pose)[2]
    #     if new_z_ang < -np.pi: new_z_ang+= np.pi
    #     if new_z_ang > np.pi: new_z_ang-= np.pi
    #     quarterion = tf.transformations.quaternion_from_euler(0,0,new_z_ang)
    #     self.goal_pose.orientation.x = quarterion[0]
    #     self.goal_pose.orientation.y = quarterion[1]
    #     self.goal_pose.orientation.z = quarterion[2]
    #     self.goal_pose.orientation.w = quarterion[3]
        

    def current_pose_callback(self, current_pose):
        self.current_pose = current_pose

        self.current_pose_np = ros_numpy.numpify(current_pose)
        self.current_z_ang = self.euler_from_pose(current_pose)[2]

        if self.last_time is None:
            self.last_time = rospy.Time.now()
            self.last_pose = self.current_pose 
            self.last_z_ang = self.current_z_ang 
            future_pose = self.current_pose

            self.v_yaw, self.v_x, self.v_y, self.v_z = 0,0,0,0
        else:
            t = rospy.Time.now()
            dt = (t - self.last_time).to_sec()
            self.last_time = t

            alpha = 0.0 # inpuse filter
            self.v_yaw = (self.current_z_ang - self.last_z_ang)/dt * (1-alpha) + self.v_yaw * alpha
            self.v_x = (current_pose.position.x - self.last_pose.position.x)/dt * (1-alpha) + self.v_x * alpha
            self.v_y = (current_pose.position.y - self.last_pose.position.y)/dt * (1-alpha) + self.v_y * alpha
            self.v_z = (current_pose.position.z - self.last_pose.position.z)/dt * (1-alpha) + self.v_z * alpha

            self.latency = 0.00 #s
            #predict future pose considering a fixed latency
            future_pose = Pose()
            future_yaw = self.current_z_ang + self.v_yaw * self.latency
            future_pose.position.x = current_pose.position.x + self.v_x * self.latency +0.5*self.v_yaw *self.v_x*self.latency**2
            future_pose.position.y = current_pose.position.y + self.v_y * self.latency +0.5*self.v_yaw *self.v_y*self.latency**2
            future_pose.position.z = current_pose.position.z + self.v_z * self.latency +0.5*self.v_yaw *self.v_z*self.latency**2
            future_pose.orientation = ros_numpy.msgify(Quaternion, tf.transformations.quaternion_from_euler(0,0,future_yaw))

            self.last_pose = current_pose
            self.last_z_ang = self.current_z_ang
            
        self.positioning_vel = self.calculate_vel(future_pose)       


    def velocity_callback(self, goal_vec): #Point
        vel = Twist()
        vel.linear.x = max(0.2,goal_vec.x)
        vel.linear.y = max(0.3,goal_vec.y)
        vel.linear.z = max(0.5,goal_vec.z)
        self.setpoint_velocity_pub.publish(vel)
        rospy.loginfo("got velocity goal")

    def parameters_callback(self, config, level):

        rospy.loginfo("""Reconfigure Request: \n Running: {running}\n P_x {P_x} I_x {I_x} D_x {D_x} \n P_y {P_y} I_y {I_y} D_y {D_y} \n P_z {P_z} I_z {I_z} D_z {D_z} \n P_ang {P_ang} I_ang {I_ang} D_ang {D_ang} \n
                                        \n mult_P_x {mult_P_x} mult_I_x {mult_I_x} mult_D_x {mult_D_x} \n mult_P_y {mult_P_y} mult_I_y {mult_I_y} mult_D_y {mult_D_y} \n mult_P_z {mult_P_z} mult_I_z {mult_I_z} mult_D_z {mult_D_z}\n mult_P_ang {mult_P_ang} mult_I_ang {mult_I_ang} mult_D_ang {mult_D_ang}""".format(**config))

        self.pid_x.set_PID_constants(config.P_x*10**config.mult_P_x, config.I_x*10**config.mult_I_x, config.D_x*10**config.mult_D_x)
        self.pid_y.set_PID_constants(config.P_y*10**config.mult_P_y, config.I_y*10**config.mult_I_y, config.D_y*10**config.mult_D_y)
        self.pid_z.set_PID_constants(config.P_z*10**config.mult_P_z, config.I_z*10**config.mult_I_z, config.D_z*10**config.mult_D_z)
        self.pid_ang.set_PID_constants(config.P_ang*10**config.mult_P_ang, config.I_ang*10**config.mult_I_ang, config.D_ang*10**config.mult_D_ang)
        self.running = config.running
        return config

    # ------- service handles ----------
    def set_calibrate_pid(self, request):
        assert isinstance(request, SetBoolRequest)
        self.calibrate_pid = request.data
        srv = Server(ControlConfig, self.parameters_callback)
        return SetBoolResponse(True, "calibrate_pid is now : "+str(self.calibrate_pid))

    # ------ control methods -----------
    def pickup_box(self,data):
        """performs the fast routine of diving, going front and up to pick up a box"""
        rospy.loginfo("PICKUP BOX")
        vel = Twist()
        vel.linear.x = 1.5
        vel.linear.y = 0
        vel.linear.z = 0
        self.setpoint_velocity_pub.publish(vel)
        rospy.sleep(0.25)

        vel.linear.x = 0 #stop
        vel.linear.y = 0
        vel.linear.z = 0
        self.setpoint_velocity_pub.publish(vel)

    def euler_from_pose(self, pose):
        quarterion = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
        return tf.transformations.euler_from_quaternion(quarterion)


    def align_camera(self):
        self.camera_angle.angular.x = 0
        self.camera_angle.angular.y = 3
        self.camera_angle.angular.z = 0
        self.camera_angle_pub.publish(self.camera_angle)

    def calculate_vel(self,pose):

        v_x_raw, v_y_raw, v_z, v_ang = self.pid_update(pose)
        # last_v_x = self.positioning_vel[0] #last x vel commnad
        # last_v_y = self.positioning_vel[1]  #last y vel commnad
        last_ang_vel_z = self.positioning_vel[3] #last angular vel commnad
        

        ang_z = self.euler_from_pose(pose)[2]
        
        if np.abs(last_ang_vel_z) < 1e-3: 
            v_x = np.cos(ang_z)*v_x_raw+np.sin(ang_z)*v_y_raw
            v_y = np.cos(ang_z)*v_y_raw-np.sin(ang_z)*v_x_raw
        else:
            # TODO: correct drift,  correct velocity commands for higher angular speeds
            v_x = np.cos(ang_z)*v_x_raw+np.sin(ang_z)*v_y_raw
            v_y = np.cos(ang_z)*v_y_raw-np.sin(ang_z)*v_x_raw
    
        # if self.count_aligned > 3:
        # if self.aligned_xyz:
        #     v_x = 0.0
        #     v_y = 0.0
        #     v_z = 0.0
        #     # pass
        # else:
        #     v_ang = 0.0
        return [v_x, v_y, v_z, v_ang]
    def reset_pid(self):
        # self.pid_setpoint(self.current_pose)
        self.pid_x.setIntegrator(0)
        self.pid_y.setIntegrator(0)
        self.pid_z.setIntegrator(0)
        self.pid_ang.setIntegrator(0)
        

    def pid_setpoint(self, goal_pose):
        self.pid_x.setPoint(goal_pose.position.x)
        self.pid_y.setPoint(goal_pose.position.y)
        self.pid_z.setPoint(goal_pose.position.z)
        ang_z = self.euler_from_pose(goal_pose)[2]
        self.pid_ang.setPoint(ang_z)

    def pid_update(self, new_pose):
        #linear update
        vel_x = self.pid_x.update(new_pose.position.x)
        vel_y = self.pid_y.update(new_pose.position.y)
        vel_z = self.pid_z.update(new_pose.position.z)
        #angular update
        new_ang_z = self.euler_from_pose(new_pose)[2]
        #check shortest way
        goal_ang = self.pid_ang.getPoint() 
        if new_ang_z - goal_ang > np.pi:
            new_ang_z -= 2*np.pi
        elif new_ang_z - goal_ang < -np.pi:
            new_ang_z += 2*np.pi
        vel_ang = self.pid_ang.update(new_ang_z)
        return np.array([vel_x,vel_y,vel_z,vel_ang])
    def pid_get_error(self):
        return [self.pid_x.getError(),self.pid_y.getError(),self.pid_z.getError(),self.pid_ang.getError()]

    def check_aligment(self):
        e = self.pid_get_error()

        p_error = Pose()
        p_error.position.x, p_error.position.y, p_error.position.z, p_error.orientation.z = e
        self.error_pose_pub.publish(p_error)
        if abs(e[0]) > self.precision[0] or abs(e[1]) > self.precision[1] or abs(e[2]) > self.precision[2]:
            self.count_aligned = 0
        else:
            self.count_aligned += 1
            # print(e)

        if self.count_aligned > 20:
            self.aligned_xyz = True

        if self.count_aligned > 20 and abs(e[3]) < self.precision[3]:

            rospy.loginfo("ALIGNED!!")
            self.aligned.publish(True)
            self.count_aligned = 0
            self.aligned_xyz = False
    

    def publish_velocity_commands(self):
        # print("closed loop")
        adjusted_vel = Twist()
        max_vel = self.max_speed

        adjusted_vel.linear.x = min(max_vel,max(self.positioning_vel[0],-max_vel))
        adjusted_vel.linear.y = min(max_vel,max(self.positioning_vel[1],-max_vel))
        adjusted_vel.linear.z = min(2*max_vel,max(self.positioning_vel[2],-2*max_vel))
        
        adjusted_vel.angular.z = self.positioning_vel[3]
        

        self.setpoint_velocity_pub.publish(adjusted_vel)
    def run(self):

        moveby_self = False
        while not rospy.is_shutdown():
            if self.running:
                
                
                self.check_aligment()
                
                #control the drone with open loop control, but use the pid to check if the drone is aligned and refine its position
                if self.openloop_supervised: 
                    max_exec_time = 10.0


                    #if the goal pose is updated, use open loop control
                    if self.last_goal_pose != self.goal_pose: 
                        self.last_goal_pose = self.goal_pose

                        delta_distance = Twist()
                        delta_distance.linear.x = self.goal_pose.position.x - self.current_pose.position.x
                        delta_distance.linear.y = self.goal_pose.position.y - self.current_pose.position.y
                        delta_distance.linear.z = -(self.goal_pose.position.z - self.current_pose.position.z)
                        delta_distance.angular.z = -(self.euler_from_pose(self.goal_pose)[2] - self.euler_from_pose(self.current_pose)[2])


                        max_exec_time = np.sqrt(delta_distance.linear.x**2+delta_distance.linear.y**2+delta_distance.linear.z**2)/0.25 + 3.0
                        print("open loop")
                        print("------------------ exec time: ", max_exec_time, "------------------")
                        print(delta_distance)

                        # rospy.sleep(0.2)
                        for i in range(1):
                            self.setpoint_moveby_pub.publish(delta_distance)
                            self.rate.sleep()

                    
                        moveby_self = True
                        self.last_goal_pose_time = rospy.Time.now()
                    
                    #if the goal pose is not updated for more than 'max_exec_time' seconds, use closed loop control
                    if moveby_self and (rospy.Time.now() - self.last_goal_pose_time).to_sec() > max_exec_time: 
                        self.publish_velocity_commands()
                        rospy.loginfo((rospy.Time.now() - self.last_goal_pose_time).to_sec())
                        # self.aligned.publish(True)
                else:

                    self.publish_velocity_commands()

                # print(adjusted_vel)
                # print(self.pid_get_error())
            self.current_pose_pub.publish(self.current_pose)
            self.rate.sleep()


if __name__ == "__main__":
    c = vso_controler()
    c.run()
