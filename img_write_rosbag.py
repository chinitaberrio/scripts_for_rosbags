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

    # initialize ros
    rospy.init_node('img_write')

    frame_info = '/gmsl/' + args["video_file"][-7:-5] + '/frame_info'
    camera_info = '/gmsl/' + args["video_file"][-7:-5] + '/camera_info'
    publish_topic = '/gmsl/' + args["video_file"][-7:-5] + '/image_color'

    pubs = {}
    tf_static_msg = None
    camera_info_flag = False
    image_counter = 0

    # start reading rosbags for frame_info
    bag = rosbag.Bag(args["bag_file"])
    output_bag_name = os.path.join(args["output_bag_path"], 'output.bag')
    output_bag = rosbag.Bag(output_bag_name,'w')
    pub_img = rospy.Publisher(publish_topic, Image, queue_size=10)
    rospy.loginfo('Start read')

    for topic, msg, t in bag.read_messages():

        if args["output_bag_path"]:
            output_bag.write(topic, msg, t)

        # publish image messages read from h264 files
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
                img_msg = bridge.cv2_to_imgmsg(image_frame, encoding="bgr8")
                img_msg.header.frame_id = 'camera_link'
                img_msg.header.stamp = msg.header.stamp
                pub_img.publish(img_msg)
                if args["output_bag_path"]:
                    output_bag.write(publish_topic, img_msg, msg.header.stamp)
                image_counter = image_counter + 1


        if rospy.is_shutdown():
            break

    bag.close()
    output_bag.close()
    cap.release()
    rospy.loginfo('Done read')



