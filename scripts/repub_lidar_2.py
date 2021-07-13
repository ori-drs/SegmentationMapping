#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import *
import cv2
import numpy as np
import math

global last_t
last_t = None


def handle_msg(msg):
    global last_t
    if last_t is None:
        return
    print("msg with shifted timestamp")
    # print "x"
    # print last_t.secs, ", ", last_t.nsecs
    msg.header.stamp = last_t
    # print msg.header.stamp.secs, ", ", msg.header.stamp.nsecs

    # print "time_shifted:" + str(msg.header.stamp.secs)
    pub.publish(msg)


def handle_msg2(msg):
    global last_t
    print("msg with j/s")
    # msg.header.stamp.secs = msg.header.stamp.secs + offset;
    last_t = msg.header.stamp

    # print "time_shifted:" + str(msg.header.stamp.secs)
    # pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("repub_lidar2")
    print("repub_lidar2")
    rospy.Subscriber("/os_cloud_node/points", PointCloud2, handle_msg)
    rospy.Subscriber("/joint_states", JointState, handle_msg2)
    pub = rospy.Publisher(
        "/os_cloud_node/points_timestamp_corrected", PointCloud2, queue_size=10, latch=True
    )
    rospy.spin()
