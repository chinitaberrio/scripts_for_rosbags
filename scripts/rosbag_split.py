import rosbag
import argparse
import os, sys
import rospy

if __name__ == '__main__':
    # necessary input args
    ap = argparse.ArgumentParser()
    ap.add_argument("-b", "--bag", required=True, help="input bag file")
    ap.add_argument("-m", "--topic", required=True, help="input topic")
    ap.add_argument("-n", "--n_messages", required=True, help="input number of messages")
    args = vars(ap.parse_args())

    # check if bag exists
    if args["bag"] is None:
        rospy.logerr("No Bag specified!")
        exit(1)
    
    # get rosbag file path and name
    bag_path = os.path.abspath(args["bag"])
    bag_name = os.path.basename(bag_path)
       
    bagfile = rosbag.Bag(bag_path)
    messages = bagfile.get_message_count(args["topic"])
    m_per_chunk = 200 #int(args["n_messages"])
    chunk = 0
    m = 1

    # output bag path and name
    output_bag_name = "chunk_%04d.bag" % chunk
    output_bag_path = os.path.join(os.path.dirname(bag_path), output_bag_name)
    print("Write new bag into: " + output_bag_path)
    outbag = rosbag.Bag(output_bag_path, 'w')

    for topic, msg, t in bagfile.read_messages():
        if topic == args["topic"]:
            m += 1
            print(topic)
            print(m)
            if m % m_per_chunk == 0:
                outbag.close()
                chunk += 1
                output_bag_name = "chunk_%04d.bag" % chunk
                output_bag_path = os.path.join(os.path.dirname(bag_path), output_bag_name)
                print("Write new bag into: " + output_bag_path)
                outbag = rosbag.Bag(output_bag_path, 'w')
        outbag.write(topic, msg, t)
    outbag.close()
    rospy.loginfo('Done writing new bag')