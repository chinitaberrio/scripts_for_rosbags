#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from tf2_msgs.msg import TFMessage
from jsk_recognition_msgs.msg import BoundingBoxArray
import message_filters


def cb_2(msg):
    msg.header.stamp = rospy.Time.now()
    pub_2.publish(msg)

def cb_3(msg):
    msg.header.stamp = rospy.Time.now()
    pub_3.publish(msg)

def cb_4(msg):
    msg.header.stamp = rospy.Time.now()
    pub_4.publish(msg)


rospy.init_node('hoge')
first_stamp = None

rospy.sleep(1)
now = rospy.Time.now()

pub_2 = rospy.Publisher('/ibeo/lidar/static', PointCloud2, queue_size=1)
pub_3 = rospy.Publisher('/ibeo/lidar/dynamic', PointCloud2, queue_size=1)
pub_4 = rospy.Publisher('/ouster/lidar_point_cloud', PointCloud2, queue_size=1)

sub_2 = rospy.Subscriber('/ibeo_interface_node/ibeo/lidar/dynamic', PointCloud2, cb_2)
sub_3 = rospy.Subscriber('/ibeo_interface_node/ibeo/lidar/static', PointCloud2, cb_3)
sub_4 = rospy.Subscriber('/ouster/points', PointCloud2, cb_4)

rospy.spin()
