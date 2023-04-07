#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from drone_control.cfg import ControlConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request:
P_x {P_x} I_x {I_x} D_x {D_x}
P_y {P_y} I_y {I_y} D_y {D_y}
P_z {P_z} I_z {I_z} D_z {D_z}
    
{str_param}, {bool_param}, {size}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("control", anonymous = False)

    srv = Server(ControlConfig, callback)
    rospy.spin()