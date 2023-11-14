import rosbag
import argparse
import os, sys
import rospy
import yaml

if __name__ == '__main__':
    # necessary input args
    ap = argparse.ArgumentParser()
    ap.add_argument("-b", "--bag", required=True, help="input bag file")
    ap.add_argument("-config", "--config_file", required=True, help=".yaml configuration file")
    args = vars(ap.parse_args())
 
    # check if bag exists
    if args["bag"] is None:
        rospy.logerr("No Bag specified!")
        exit(1)
    if args["config_file"] is None:
        rospy.logerr("No configuration file specified!")
        exit(1)

    # get rosbag file path and name
    bag_path = os.path.abspath(args["bag"])
    bag_name = os.path.basename(bag_path)
    
    output_bag_name = bag_name.replace(".bag","_tf_static_fixed.bag")
    
    with open(args["config_file"], "r") as stream:
        try:
            new_tf_static = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    
    camera_list = list(new_tf_static.keys())
    print (camera_list)

    # output bag path and name
    output_bag_path = os.path.join(os.path.dirname(bag_path), output_bag_name)
    print("Write new bag into: " + output_bag_path)

    with rosbag.Bag(output_bag_path , 'w') as outbag:
        for topic, msg, t in rosbag.Bag(bag_path).read_messages():
            # This also replaces tf timestamps under the assumption 
            # that all transforms in the message share the same timestamp
            if (topic == "/tf" or topic == "/tf_static") and msg.transforms:
                if msg.transforms[0].child_frame_id in camera_list:
                    print ('changing tf ')
                    print (msg.transforms[0].child_frame_id)
                    msg.transforms[0].header.frame_id = "os_sensor"
                    msg.transforms[0].transform.translation.x = new_tf_static[msg.transforms[0].child_frame_id]['trans_x']
                    msg.transforms[0].transform.translation.y = new_tf_static[msg.transforms[0].child_frame_id]['trans_y']
                    msg.transforms[0].transform.translation.z = new_tf_static[msg.transforms[0].child_frame_id]['trans_z']
                    msg.transforms[0].transform.rotation.x = new_tf_static[msg.transforms[0].child_frame_id]['rot_x']
                    msg.transforms[0].transform.rotation.y = new_tf_static[msg.transforms[0].child_frame_id]['rot_y']
                    msg.transforms[0].transform.rotation.z = new_tf_static[msg.transforms[0].child_frame_id]['rot_z']
                    msg.transforms[0].transform.rotation.w = new_tf_static[msg.transforms[0].child_frame_id]['rot_w']
                outbag.write(topic, msg, t)
            else:
                outbag.write(topic, msg, t)

    outbag.close()
    rospy.loginfo('Done writing new bag')