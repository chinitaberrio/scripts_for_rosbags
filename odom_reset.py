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
    ap.add_argument("-odom", "--odom_topic", default='/ibeo_interface_node/ibeo/odometry', help="odometry topic")
    args = vars(ap.parse_args())

    # check if bag exists
    if args["bag"] is None:
        rospy.logerr("No Bag specified!")
        exit(1)

    # initialize classes
    rospy.init_node('odom_reset', anonymous=True)
    
    odom_topic = os.path.abspath(args["odom_topic"])
    odom_msg = None
    odom_counter = 0
    odom_rot_mat = None
    odom_trans = [0, 0, 0]
   
    # get rosbag file path and name
    bag_path = os.path.abspath(args["bag"])
    bag_name = os.path.basename(bag_path)
    output_bag_name = bag_name.replace(".bag","_fixed_odom.bag")
    
    # output bag path and name
    output_bag_path = os.path.join(os.path.dirname(bag_path), output_bag_name)
    print("Write new bag into: " + output_bag_path)

    # get rosbag info
    bag = rosbag.Bag(args["bag"])
    rospy.loginfo('Start reading bag ' + args["bag"])
    output_bag = rosbag.Bag(output_bag_path, 'w')

    # start processing when play back the rosbag
    for topic, msg, t in bag.read_messages():

        # read topics in the rosbag
        if topic == odom_topic:
            if odom_counter == 0:
                odom_rot_mat = quaternion_matrix([msg.pose.pose.orientation.x,
                                                   msg.pose.pose.orientation.y,
                                                   msg.pose.pose.orientation.z,
                                                   msg.pose.pose.orientation.w])

                odom_trans[0] = msg.pose.pose.position.x
                odom_trans[1] = msg.pose.pose.position.y
                odom_trans[2] = msg.pose.pose.position.z
                odom_msg = replace_odometry(msg,odom_rot_mat, odom_trans)
                odom_counter = odom_counter+1
            else:
                odom_msg = replace_odometry(msg, odom_rot_mat, odom_trans)
                odom_counter = odom_counter + 1
            output_bag.write(odom_topic, odom_msg, t)

        # rewrite the rosbag
        else:
            output_bag.write(topic, msg, t)

        if rospy.is_shutdown():
            break

    bag.close()
    output_bag.close()
    rospy.loginfo('Done writing new bag')
