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
    ap.add_argument("-r", "--rectify", default=False, help="Whether to rectify the extracted images.")
    ap.add_argument("-d", "--divider", default=1, type=int, help="save image every DIVIDER frames")
    ap.add_argument("-s", "--save_dir", default=None, help="save the extracted images")
    args = vars(ap.parse_args())

    if args["save_dir"] and not os.path.exists(args["save_dir"]):
        os.makedirs(args["save_dir"])

    cap = cv2.VideoCapture(args["video_file"])
    bridge = CvBridge()

    # initialize ros
    rospy.init_node('img_extraction')

    frame_info = '/sekonix_camera/' + args["video_file"][-16:-4] + '/frame_info'
    camera_info = '/sekonix_camera/' + args["video_file"][-16:-4] + '/camera_info'

    print(frame_info)

    tf_static_msg = None
    camera_info_flag = False
    cameraMatrix = None
    distCoeffs = None
    camera_model = None
    output_img = None
    camera_D = None
    DIM = None
    image_counter = 0

    # start reading rosbags for frame_info
    bag = rosbag.Bag(args["bag_file"])
    rospy.loginfo('Start read')

    for topic, msg, t in bag.read_messages():

        if topic == camera_info and camera_info_flag == False:
            cameraMatrix = np.reshape(msg.K, (3, 3))
            distCoeffs = np.array(msg.D)
            camera_model = msg.distortion_model
            camera_D = np.reshape(distCoeffs, (-1, 1))
            camera_info_flag = True
            DIM = [msg.width, msg.height]

        if topic == frame_info and camera_info_flag:

            while image_counter < msg.frame_counter:
                ret, image_frame = cap.read()
                if ret == False:
                    break
                image_counter = image_counter + 1

            if image_counter == msg.frame_counter:

                ret, image_frame = cap.read()

                if ret == False:
                    break

                if args["rectify"]:

                    if camera_model == "plumb_bob":
                        output_img = cv2.undistort(image_frame, cameraMatrix, distCoeffs)

                    elif camera_model == "equidistant":
                        dim1 = image_frame.shape[:2][::-1] 
                        scaled_K = cameraMatrix * dim1[0] / DIM[0]  
                        scaled_K[2][2] = 1.0  
                        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, camera_D, dim1, np.eye(3),
                                                                                       balance=0)
                        map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, camera_D, np.eye(3), new_K, dim1,
                                                                         cv2.CV_16SC2)
                        output_img = cv2.remap(image_frame, map1, map2, interpolation=cv2.INTER_LINEAR,
                                                    borderMode=cv2.BORDER_CONSTANT)

                else:
                    output_img = image_frame

                if args["save_dir"] and (msg.frame_counter % int(args["divider"])) == 0:

                    cv2.imwrite(os.path.join(args["save_dir"],  str(
                                                 msg.header.stamp) + '.png'), output_img)
                    print(str("%08d" % msg.frame_counter) + '.png')

                image_counter = image_counter + 1

        if rospy.is_shutdown():
            break

    bag.close()
    cap.release()
    rospy.loginfo('Done read')



