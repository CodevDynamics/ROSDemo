#! /usr/bin/env python

import random
import rospy
from codevqtros_msgs.msg import SetObject, DetectObject

def set_object_cb(msg):
    rospy.loginfo("Received Set Object Message from Fly: (%f, %f, %f, %f)", msg.x, msg.y, msg.width, msg.height)

if __name__ == "__main__":
    rospy.init_node("detect_node_py")

    set_object_sub = rospy.Subscriber("codevqtros/detect/SetObject", SetObject, callback = set_object_cb)
    detect_object_pub = rospy.Publisher("codevqtros/detect/Objects", DetectObject, queue_size=20)

    rate = rospy.Rate(20)
    count = 0;

    while(not rospy.is_shutdown()):
        if rospy.get_param('codevqtros/detect/enabled'):
            x = random.uniform(0, 100) / 100.0
            y = random.uniform(0, 100) / 100.0

            msg = DetectObject()
            if count >= 20:
                count = 0;
                msg.x = 0;
                msg.y = 0;
                msg.width = 0;
                msg.height = 0;
            else:
                msg.x = x
                msg.y = y
                msg.width = 0.1
                msg.height = 0.3
            msg.objectId = "Apple";
            msg.rgba = "#ff00ff00";
            count = count + 1
            detect_object_pub.publish(msg)

        rate.sleep()