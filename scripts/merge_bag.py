#!/usr/bin/env python

import sys
import argparse
import glob 
from rosbag import Bag

def main():

    ap = argparse.ArgumentParser(description='Merge bag files in a folder')
    ap.add_argument("-bs", "--bag_files", required=True, help="The directory for bag files")
    args = vars(ap.parse_args())

    bags_list = glob.glob(args["bag_files"]+'/*bag')
    bags_list.sort()
    print(bags_list)

    output_bag = bags_list[0].replace("0.bag","merged.bag")
    print(output_bag)

    with Bag(output_bag, 'w') as o: 
        for ifile in bags_list:
            with Bag(ifile, 'r') as ib:
                for topic, msg, t in ib:
                    o.write(topic, msg, t)

    o.close()

if __name__ == "__main__":
    main()
