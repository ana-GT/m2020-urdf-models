#! /usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import JointState



def BladeSpinner(ns = ""):

    rpm = 2400
    update_rate = 0.01
    rad2rev = (2*3.14)

    # rev/min * rad/rev = rad/min
    # rad/min * min/sec = rad/sec
    # rad/sec * sec/inc = rac/inc

    blade_inc = update_rate*2400*rad2rev/60

    pub = rospy.Publisher(ns + 'joint_state_controller/joint_command', JointState, queue_size=10)
    rospy.init_node('blade_spinner', anonymous=True)
    rate = rospy.Rate(1/update_rate)

    js = JointState()
    js.name = ['MHS_TopBlades_v16', 'MHS_BottomBlades_v16']
    js.position = [0.0, 0.0]

    while not rospy.is_shutdown():    
        js.position[0] = js.position[0]+blade_inc
        js.position[1] = js.position[1]-blade_inc

        if js.position[0] > 3.14 :
            js.position[0] = -3.14

        if js.position[1] < -3.14 :
            js.position[1] = 3.14

        pub.publish(js)
        rate.sleep()

if __name__ == '__main__':

    ns = ""
    if len(sys.argv) > 1:
        ns = sys.argv[1] + "/"

    try:
        BladeSpinner(ns)
    except rospy.ROSInterruptException:
        pass


