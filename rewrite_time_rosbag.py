import rosbag
import argparse
import os, sys
import rospy

if __name__ == '__main__':
    # necessary input args
    ap = argparse.ArgumentParser()
    ap.add_argument("-b", "--bag", required=True, help="input bag file")
    args = vars(ap.parse_args())

    # check if bag exists
    if args["bag"] is None:
        rospy.logerr("No Bag specified!")
        exit(1)
    
    # get rosbag file path and name
    bag_path = os.path.abspath(args["bag"])
    bag_name = os.path.basename(bag_path)
    
    output_bag_name = bag_name.replace(".bag","_msg_timings.bag")
    
    # output bag path and name
    output_bag_path = os.path.join(os.path.dirname(bag_path), output_bag_name)
    print("Write new bag into: " + output_bag_path)

    with rosbag.Bag(output_bag_path , 'w') as outbag:
        for topic, msg, t in rosbag.Bag(bag_path).read_messages():
            # This also replaces tf timestamps under the assumption 
            # # that all transforms in the message share the same timestamp
            if topic == "/tf" and msg.transforms:
                outbag.write(topic, msg, msg.transforms[0].header.stamp)
            else:
                outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)

    outbag.close()
    rospy.loginfo('Done writing new bag')
