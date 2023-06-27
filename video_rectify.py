#!/usr/bin/python
import numpy as np
import cv2
import argparse
import rosbag


if __name__ == '__main__':

    ap = argparse.ArgumentParser()
    ap.add_argument("-b", "--bag_file", required=True, help="The rosbag you would like to extract images")
    ap.add_argument("-v", "--video", required=True, help="The directory for mp4 video file")
    ap.add_argument("-o", "--output_file", default='/home/user/Downloads/video.avi', help="save the rectified videos")
    args = vars(ap.parse_args())

    if not args["video"]:
        print("No video specified")
    else:
        cap = cv2.VideoCapture(args["video"])


    bag = rosbag.Bag(args["bag_file"])
    camera_info = '/gmsl/' + args["video"][-7:-5] + '/camera_info'


    camera_info_flag = False
    cameraMatrix = None
    distCoeffs = None
    camera_model = None

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(args["output_file"], fourcc, 30.0, (1920, 1208))

    for topic, msg, t in bag.read_messages():

        if topic == camera_info and camera_info_flag == False:
            cameraMatrix = np.reshape(msg.K, (3, 3))
            distCoeffs = np.array(msg.D)
            camera_D = np.reshape(distCoeffs, (4, 1))
            camera_info_flag = True
            camera_model = msg.distortion_model
            DIM = [msg.width, msg.height]
            break

        if camera_info_flag == True:
            break

    if camera_info_flag is True:

        while (cap.isOpened()):
            ret, frame = cap.read()

            if ret==False:
                break

            if camera_model == "plumb bob":
                        output_img = cv2.undistort(frame, cameraMatrix, distCoeffs)
            if camera_model == "equidistant":
                dim1 = frame.shape[:2][::-1]  # dim1 is the dimension of input image to un-distort
                scaled_K = cameraMatrix * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
                scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
                # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
                new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, camera_D, dim1, np.eye(3),
                                                                            balance=0)
                map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, camera_D, np.eye(3), new_K, dim1,
                                                                cv2.CV_16SC2)
                output_img = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR,
                                    borderMode=cv2.BORDER_CONSTANT)

            # write the rectified frame
            out.write(output_img)


    # Release everything if job is finished
    cap.release()
    out.release()
