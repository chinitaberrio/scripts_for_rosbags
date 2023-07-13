import rospy
import rosbag
import argparse
import os, sys
from tf.msg import tfMessage
from tf2_msgs.msg import TFMessage
import tf_conversions
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

def add_tfstatic(msg):
    #hardcoded values at the moment
    tf2_lidar = geometry_msgs.msg.TransformStamped()
    tf2_lidar.header.frame_id = "base_link"
    tf2_lidar.child_frame_id = "os_sensor"
    tf2_lidar.header.stamp = msg.header.stamp
    tf2_lidar.transform.translation.x = 0.79
    tf2_lidar.transform.translation.y = 0
    tf2_lidar.transform.translation.z = 1.7
    tf2_lidar.transform.rotation.x = 0.01
    tf2_lidar.transform.rotation.y = 0.005
    tf2_lidar.transform.rotation.z = -0.00005
    tf2_lidar.transform.rotation.w = 0.999937
    msg.transforms.append(tf2_lidar)

    return msg

if __name__ == '__main__':

    # necessary input args
    ap = argparse.ArgumentParser()
    ap.add_argument("-b", "--bag", required=True, help="input bag file")
    ap.add_argument("-odom", "--odom_topic", default='/ibeo_interface_node/ibeo/odometry', help="odometry topic")
    ap.add_argument("-odom_tf", "--odom_tf", default=False, help="odometry topic")
    ap.add_argument("-add_tf_s", "--add_tf_static", default=False, help="odometry topic")
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
    tf_odom = geometry_msgs.msg.TransformStamped()
    tf_odom.header.frame_id = "odom_init"
    tf_odom.child_frame_id = "base_link"
    tf_static_msg_queue = deque()
    tf_static_msg = None
    first = True
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

            if (args["odom_tf"]):
                tf_msg = tfMessage()   
                tf_odom.header.stamp = odom_msg.header.stamp
                tf_odom.transform.translation.x = odom_msg.pose.pose.position.x
                tf_odom.transform.translation.y = odom_msg.pose.pose.position.y
                tf_odom.transform.translation.z = odom_msg.pose.pose.position.z
                tf_odom.transform.rotation.x = odom_msg.pose.pose.orientation.x
                tf_odom.transform.rotation.y = odom_msg.pose.pose.orientation.y
                tf_odom.transform.rotation.z = odom_msg.pose.pose.orientation.z
                tf_odom.transform.rotation.w = odom_msg.pose.pose.orientation.w
                tf_msg.transforms.append(tf_odom)
                output_bag.write('/tf', tf_msg, t)
        else:
            output_bag.write(topic, msg, t)

        if first and args["add_tf_static"]:  # This needs a config file
            tf2_msg = TFMessage()
            tf2_lidar = geometry_msgs.msg.TransformStamped()
            tf2_lidar.header.frame_id = "base_link"
            tf2_lidar.child_frame_id = "os_sensor"
            tf2_lidar.header.stamp = msg.header.stamp
            tf2_lidar.transform.translation.x = 0.79
            tf2_lidar.transform.translation.y = 0
            tf2_lidar.transform.translation.z = 1.7
            tf2_lidar.transform.rotation.x = 0.01
            tf2_lidar.transform.rotation.y = 0.005
            tf2_lidar.transform.rotation.z = -0.00005
            tf2_lidar.transform.rotation.w = 0.999937
            tf2_msg.transforms.append(tf2_lidar)
            output_bag.write('/tf_static', tf2_msg, t)
            first = False

        #elif topic == '/tf_static':
            #adding the missing transformations
        #    tf2_msg = add_tfstatic(msg)
        #    output_bag.write('/tf_static', tf2_msg, t)
        # rewrite the rosbag

        if rospy.is_shutdown():
            break


    bag.close()
    output_bag.close()
    rospy.loginfo('Done writing new bag')
