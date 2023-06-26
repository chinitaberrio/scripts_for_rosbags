import rospy
import rosbag
import argparse
import os
import yaml
from scipy.stats import gaussian_kde
import numpy as np

def kde_estimate_maxima(data):
    kde = gaussian_kde(data)
    no_samples = 20
    samples = np.linspace(min(data), max(data), no_samples)
    probs = kde.evaluate(samples)
    maxima_index = probs.argmax()
    maxima = samples[maxima_index]
    return maxima

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

    # initialize classes
    rospy.init_node('rosbag_health_check', anonymous=True)
   
    # get rosbag file path and name
    bag_path = os.path.abspath(args["bag"])
    bag = rosbag.Bag(args["bag"])
    rospy.loginfo('Start reading bag ' + args["bag"])

    with open(args["config_file"], "r") as stream:
        try:
            topics_config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    topics_list = list(topics_config.keys())

    found_report = dict(zip(topics_list, [False]*len(topics_list)))
    sync_times = {key: {"First": None, "Last": None, "Counter": 0} for key in topics_list}
    
    # start processing when play back the rosbag
    for topic, msg, t in bag.read_messages():
        # read topics in the rosbag
        if topic in topics_list:
            if topics_config[topic]['required_in_rosbag']:
                found_report[topic] = True
                if sync_times[topic]['First'] == None:
                        if hasattr(msg, 'transforms'):
                            sync_times[topic]['First'] = msg.transforms[0].header.stamp.secs + msg.transforms[0].header.stamp.nsecs*1e-9
                        else:
                            sync_times[topic]['First'] = (msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9) if msg._has_header else (t.secs + t.nsecs*1e-9)
                        sync_times[topic]['Counter'] = 1
                elif topics_config[topic]['required_frequency_check']:
                        if not hasattr(msg, 'transforms'):
                            sync_times[topic]['Last'] = (msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9) if msg._has_header else (t.secs+ t.nsecs*1e-9)
                            sync_times[topic]['Counter'] = sync_times[topic]['Counter'] + 1

    start_times = [inner_dict['First'] for inner_dict in sync_times.values()]
    start_times_filter = list(filter(lambda item: item is not None, start_times))
    start_time = kde_estimate_maxima(np.array(start_times_filter))

    for topic in topics_list:
        print(topic)
        if topics_config[topic]['required_in_rosbag'] and found_report[topic]:
            if topics_config[topic]['required_frequency_check']:
                frequency = sync_times[topic]['Counter']/(sync_times[topic]['Last'] - sync_times[topic]['First'])
                print(frequency)
                if frequency - float(topics_config[topic]['rate'])  > 0.1:
                    print ('Topic {} has missing data or frequency is not correct!'.format(topic))

            if abs (sync_times[topic]['First'] - start_time) > 5.0:
                print ('Topic {} is out of sync!'.format(topic)) 
        else:
            print ('Topic {} is required, but it was not found in the rosbag!'.format(topic))

    bag.close()
    rospy.loginfo('End of the health check')
