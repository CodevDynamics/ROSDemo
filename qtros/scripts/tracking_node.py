#! /usr/bin/env python

import random
import rospy
from codevqtros_msgs.msg import SetObject, TrackingObject

def set_object_cb(msg):
    rospy.loginfo("Received Set Object Message from Fly: (%f, %f, %f, %f)", msg.x, msg.y, msg.width, msg.height)

if __name__ == "__main__":
    rospy.init_node("tracking_node_py")

    set_object_sub = rospy.Subscriber("codevqtros/tracking/SetObject", SetObject, callback = set_object_cb)
    tracking_object_pub = rospy.Publisher("codevqtros/tracking/Object", TrackingObject, queue_size=2)

    rate = rospy.Rate(20)
    count = 0;

    while(not rospy.is_shutdown()):
        x = random.uniform(0, 100) / 100.0
        y = random.uniform(0, 100) / 100.0

        msg = TrackingObject()
        msg.x = x
        msg.y = y
        msg.width = 0.1
        msg.height = 0.3
        msg.status = 1
        count = count + 1
        tracking_object_pub.publish(msg)

        rate.sleep()