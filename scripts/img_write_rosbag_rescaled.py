#!/usr/bin/python
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
    ap.add_argument("-b", "--bag_file", required=True, help="The directory for bag file")
    ap.add_argument("-v", "--video_file", required=True, help="The directory for mp4 video file")
    ap.add_argument("-o", "--output_bag_path", default=None, help="the directory to save the new rosbag with images in it")
    args = vars(ap.parse_args())

    if not os.path.isfile(args["video_file"]):
        print('No video specified.')
        sys.exit()

    if args["output_bag_path"] and not os.path.exists(args["output_bag_path"]):
        os.makedirs(args["output_bag_path"])

    cap = cv2.VideoCapture(args["video_file"])
    bridge = CvBridge()

    rospy.init_node('img_write')

    frame_info = '/sekonix_camera/' + args["video_file"][-16:-4] + '/frame_info'
    camera_info = '/sekonix_camera/' + args["video_file"][-16:-4] + '/camera_info'
    image_color = '/sekonix_camera/' + args["video_file"][-16:-4] + '/image_color'

    image_counter = 0
    bag = rosbag.Bag(args["bag_file"])
    output_bag_name = os.path.join(args["output_bag_path"], 'output.bag')
    output_bag = rosbag.Bag(output_bag_name,'w')

    rospy.loginfo('Start read')

    for topic, msg, t in bag.read_messages():

        if args["output_bag_path"]:
            output_bag.write(topic, msg, t)

        # publish image messages read from mp4 files
        if topic == frame_info:

            # start publishing images
            while image_counter < msg.frame_counter:
                ret, image_frame = cap.read()
                if ret == False:
                    break
                image_counter = image_counter + 1

            if image_counter > msg.frame_counter:
                continue
            elif image_counter == msg.frame_counter:

                ret, image_frame = cap.read()
                if ret == False:
                    break

                resized = cv2.resize(image_frame, (480, 302), interpolation = cv2.INTER_AREA)
                img_msg = bridge.cv2_to_imgmsg(resized, encoding="bgr8")
                img_msg.header.frame_id = msg.header.frame_id
                img_msg.header.stamp = msg.header.stamp
                if args["output_bag_path"]:
                    output_bag.write(image_color, img_msg, t)
                image_counter = image_counter + 1

        if rospy.is_shutdown():
            break

    bag.close()
    output_bag.close()
    cap.release()
    rospy.loginfo('Done read')



