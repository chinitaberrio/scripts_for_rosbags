#!/usr/bin/python
import numpy as np
import cv2
import os
import argparse
import sys
import rosbag
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


if __name__ == '__main__':

    ap = argparse.ArgumentParser()
    ap.add_argument("-b", "--bag_file", required=True, help="The rosbag you would like to extract images")
    ap.add_argument("-v", "--video_file", required=True, help="The mp4 video file for one camera")
    ap.add_argument("-o", "--output_file", default='/home/user/Downloads/video.avi', help="save the videos")
    args = vars(ap.parse_args())

    cap = cv2.VideoCapture(args["video_file"])
    bridge = CvBridge()

    # initialize ros
    rospy.init_node('extract_video')

    frame_info = '/sekonix_camera/' + args["video_file"][-16:-4] + '/frame_info'

    print(frame_info)

    image_counter = 0

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(args["output_file"], fourcc, 30.0, (1920, 1208))

    # start reading rosbags for frame_info
    bag = rosbag.Bag(args["bag_file"])
    rospy.loginfo('Start read')

    for topic, msg, t in bag.read_messages():

        if topic == frame_info :

            while image_counter < msg.frame_counter:
                ret, image_frame = cap.read()
                if ret == False:
                    break
                image_counter = image_counter + 1

            if image_counter == msg.frame_counter:

                ret, image_frame = cap.read()

                if ret == False:
                    break
                output_img = image_frame

                out.write(output_img)

                image_counter = image_counter + 1

        if rospy.is_shutdown():
            break

    bag.close()
    cap.release()
    out.release()
    rospy.loginfo('Done read')



