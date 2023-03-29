import rospy
import rosbag
import argparse
import os, sys
from tf.msg import tfMessage
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply, quaternion_inverse, \
    quaternion_matrix, quaternion_from_matrix, inverse_matrix, euler_from_matrix
from collections import deque
import geometry_msgs.msg



def replace_odometry(msg, i_rot, i_trans):

    odometry_msg = msg

    rot_mat = quaternion_matrix([msg.pose.pose.orientation.x,
                                 msg.pose.pose.orientation.y,
                                 msg.pose.pose.orientation.z,
                                 msg.pose.pose.orientation.w])

    final_rot = np.matmul(rot_mat, inverse_matrix(i_rot))

    trans = np.matmul(inverse_matrix(i_rot), [msg.pose.pose.position.x - i_trans[0],
                                              msg.pose.pose.position.y - i_trans[1],
                                              msg.pose.pose.position.z - i_trans[2],
                                              0])
    odometry_msg.pose.pose.position.x = trans[0]
    odometry_msg.pose.pose.position.y = trans[1]
    odometry_msg.pose.pose.position.z = trans[2]

    converted_quaternion = quaternion_from_matrix(final_rot)

    odometry_msg.pose.pose.orientation.x = converted_quaternion[0]
    odometry_msg.pose.pose.orientation.y = converted_quaternion[1]
    odometry_msg.pose.pose.orientation.z = converted_quaternion[2]
    odometry_msg.pose.pose.orientation.w = converted_quaternion[3]

    return odometry_msg


if __name__ == '__main__':

    # necessary input args
    ap = argparse.ArgumentParser()
    ap.add_argument("-b", "--bag", required=True, help="input bag file")
    ap.add_argument("-o", "--output_bag", required=True, help="output new bag path")
    args = vars(ap.parse_args())

    # check if bag exists
    if args["bag"] is None:
        rospy.logerr("No Bag specified!")
        exit(1)

    # check if output bag path exists
    if not os.path.exists(args["output_bag"]):
        os.mkdir(args["output_bag"])

    # initialize classes
    rospy.init_node('odom_reset', anonymous=True)
    
    odom_msg = '/ibeo_interface_node/ibeo/odometry'
    vn100_msg = None
    vn100_counter = 0
    vn100_rot_mat = None
    vn100_trans = [0, 0, 0]
   
    # get rosbag file path and name
    bag_path = os.path.abspath(args["bag"])
    bag_name = os.path.basename(bag_path)
    # output bag path and name
    output_bag_name = os.path.join(args["output_bag"], bag_name)
    print("Write new bag into: " + output_bag_name)

    # get rosbag info
    bag = rosbag.Bag(args["bag"])
    rospy.loginfo('Start reading bag ' + args["bag"])
    output_bag = rosbag.Bag(output_bag_name, 'w')

    # start processing when play back the rosbag
    for topic, msg, t in bag.read_messages():

        # read topics in the rosbag
        if topic == odom_msg:
            if vn100_counter == 0:
                vn100_rot_mat = quaternion_matrix([msg.pose.pose.orientation.x,
                                                   msg.pose.pose.orientation.y,
                                                   msg.pose.pose.orientation.z,
                                                   msg.pose.pose.orientation.w])

                vn100_trans[0] = msg.pose.pose.position.x
                vn100_trans[1] = msg.pose.pose.position.y
                vn100_trans[2] = msg.pose.pose.position.z
                vn100_msg = replace_odometry(msg,vn100_rot_mat, vn100_trans)
                vn100_counter = vn100_counter+1
            else:
                vn100_msg = replace_odometry(msg, vn100_rot_mat, vn100_trans)
                vn100_counter = vn100_counter + 1
            output_bag.write(odom_msg, vn100_msg, t)

        # not changed topic
        else:
            output_bag.write(topic, msg, t)

        if rospy.is_shutdown():
            break

