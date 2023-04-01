#! /usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import MountControl

def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z

pitch = 0.0
roll = 0.0
yaw = 0.0
gimbal_init = False

def mount_orientation_cb(msg):
    global gimbal_init,pitch,roll,yaw
    gimbal_init = True
    roll,pitch,yaw = quaternion_to_euler(msg.x, msg.y, msg.z, msg.w)
    yaw=-yaw
    rospy.loginfo("Received Mount Orientaion Message pitch, roll, yaw: (%f, %f, %f)", pitch, roll, yaw)

if __name__ == "__main__":
    rospy.init_node("gimbal_node_py")

    mount_orientation_sub = rospy.Subscriber("mavros/mount_control/orientation", Quaternion, callback = mount_orientation_cb)
    mount_control_pub = rospy.Publisher("mavros/mount_control/command", MountControl, queue_size=20)

    rate = rospy.Rate(5)
    
    while(not rospy.is_shutdown() and not gimbal_init):
        rate.sleep()

    mount_control = MountControl()
    mount_control.roll = 0
    mount_control.mode = 2
    if math.fabs(yaw) < 10:
        mount_control.yaw = 100
    else:
        mount_control.yaw = 0
    mount_control.pitch = 0;

    while(not rospy.is_shutdown()):
        mount_control_pub.publish(mount_control)

        rate.sleep()