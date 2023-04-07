#!/usr/bin/env python
from std_msgs.msg import String, Empty, Bool
import rospy
import os
import rospkg

rospack = rospkg.RosPack()
killing_file_path = str(rospack.get_path('drone_control')+"/../../kill_bebop.txt")
landing_file_path = str(rospack.get_path('drone_control')+"/../../land_bebop.txt")
hover_file_path = str(rospack.get_path('drone_control')+"/../../hover_bebop.txt")

print(killing_file_path)
def main():
    pub_reset = rospy.Publisher('/bebop/reset', Empty, queue_size=1)
    pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=1)
    pub_manual = rospy.Publisher('/control/set_manual_mode', Bool, queue_size=1)


    rospy.init_node('kill_button', anonymous=True)
    action = None
    while not rospy.is_shutdown():
        
        if os.path.exists(killing_file_path):
            if action is None:
                print("killing")
                action = "killing"
            pub_reset.publish()
        elif os.path.exists(landing_file_path):
            if action is None:
                print("landing")
                action = "landing"
            pub_land.publish()  
        elif os.path.exists(hover_file_path):
            if action is None:
                print("hover")
                action = "hover"
                pub_manual.publish(True)
                os.remove(hover_file_path)

        else:
            action = None     
        # rate.sleep()

if __name__ == '__main__':

    #delete file if exists
    if os.path.exists(killing_file_path):
        os.remove(killing_file_path)
    if os.path.exists(landing_file_path):
        os.remove(landing_file_path)
    if os.path.exists(hover_file_path):
        os.remove(hover_file_path)
    try:
        main()
    except rospy.ROSInterruptException:
        pass

