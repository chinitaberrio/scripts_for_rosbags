#!/usr/bin/python
import os, sys
import argparse
import rosbag
import rospy
import time
from novatel_oem7_msgs.msg import PositionOrVelocityType

param_topic_of_interest = '/novatel/oem7/bestpos'


class BagHelper:
    bag_opened = False
    
    def __init__(self, bag_path, topic_of_interest):
        # open rosbag
        self.bag = rosbag.Bag(bag_path)
        self.bag_opened = True
        rospy.loginfo('Bag opened: %s', bag_path)

        self.bag_connections = list(self.bag._get_connections(topics=[topic_of_interest]))
        self.bag_entries = [entry for entry in self.bag._get_entries(self.bag_connections)]
        rospy.loginfo('Loaded %d messages from topic: %s', self.get_msg_count(), topic_of_interest)

    def __del__(self):
        if self.bag_opened:
            self.bag.close()
            #rospy.loginfo('Bag closed')

    def get_msg_count(self):
        return len(self.bag_entries)

    def read_msg(self, msg_ind):
        entry = self.bag_entries[msg_ind]
        topic, msg, t = self.bag._read_message((entry.chunk_pos, entry.offset))
        return msg

class Interface:

    def __init__(self, bag_path, output_kml_path):
        self.bag_helper = BagHelper(bag_path, param_topic_of_interest)
        self.kml_path = output_kml_path

    def __del__(self):
        #rospy.loginfo('Interface terminates')
        return

    def convert2kml(self):
        print('<?xml version="1.0" encoding="UTF-8"?>\n<kml xmlns="http://www.opengis.net/kml/2.2">\n  <Document>\n    <name>Paths</name>\n    <description>INS Test</description>')
        print('    <Style id="greenLineGreenPoly">\n      <LineStyle>\n        <color>ff00ff00</color>\n        <width>6</width>\n      </LineStyle>\n      <PolyStyle>\n        <color>7f00ff00</color>\n      </PolyStyle>\n    </Style>')
        print('    <Style id="yellowLineGreenPoly">\n      <LineStyle>\n        <color>7f00ffff</color>\n        <width>6</width>\n      </LineStyle>\n      <PolyStyle>\n        <color>7f00ff00</color>\n      </PolyStyle>\n    </Style>')
        print('    <Style id="redLineRedPoly">\n      <LineStyle>\n        <color>ff0000ff</color>\n        <width>6</width>\n      </LineStyle>\n      <PolyStyle>\n        <color>ff0000ff</color>\n      </PolyStyle>\n    </Style>')
        pre_status = 999
        for msg_ind in range(0, self.bag_helper.get_msg_count()):
            self.msg = self.bag_helper.read_msg(msg_ind)
            #stdev = max(self.msg.lat_stdev, self.msg.lon_stdev)
            status_sol = int(self.msg.pos_type.type)

            if status_sol >= 52 and status_sol <= 56:
                cur_status = 0
                if pre_status != cur_status:
                    if pre_status != 999:
                        print('        </coordinates>\n      </LineString>\n    </Placemark>')
                    print('    <Placemark>\n      <styleUrl>#greenLineGreenPoly</styleUrl>\n      <LineString>\n        <extrude>1</extrude>\n        <tessellate>1</tessellate>\n        <altitudeMode>absolute</altitudeMode>\n        <coordinates> ')
                print('          {},{},{}'.format(self.msg.lon, self.msg.lat, self.msg.hgt + 10))
                pre_status = cur_status
            elif status_sol >= 34 and status_sol <= 51:
                cur_status = 1
                if pre_status != cur_status:
                    if pre_status != 999:
                        print('        </coordinates>\n      </LineString>\n    </Placemark>')
                    print('    <Placemark>\n      <styleUrl>#yellowLineGreenPoly</styleUrl>\n      <LineString>\n        <extrude>1</extrude>\n        <tessellate>1</tessellate>\n        <altitudeMode>absolute</altitudeMode>\n        <coordinates> ')
                print('          {},{},{}'.format(self.msg.lon, self.msg.lat, self.msg.hgt + 10))
                pre_status = cur_status
            else:
                cur_status = 2
                if pre_status != cur_status:
                    if pre_status != 999:
                        print('        </coordinates>\n      </LineString>\n    </Placemark>')
                    print('    <Placemark>\n      <styleUrl>#redLineRedPoly</styleUrl>\n      <LineString>\n        <extrude>1</extrude>\n        <tessellate>1</tessellate>\n        <altitudeMode>absolute</altitudeMode>\n        <coordinates> ')
                print('          {},{},{}'.format(self.msg.lon, self.msg.lat, self.msg.hgt + 10))
                pre_status = cur_status
        print('        </coordinates>\n      </LineString>\n    </Placemark>')
        print('  </Document>\n</kml>')

if __name__ == '__main__':

    # necessary input args
    ap = argparse.ArgumentParser()
    ap.add_argument("-b", "--bag", required=True, help="input bag file")
    args = vars(ap.parse_args())
  
    # get rosbag file path and name
    bag_path = os.path.abspath(args["bag"])
    bag_name = os.path.basename(bag_path)
    output_kml = bag_name.replace(".bag",".kml")
    output_kml_path = os.path.join(os.path.dirname(bag_path), output_kml)

    # initialize ros
    rospy.init_node('rtk2kml')

    interface = Interface(bag_path, output_kml_path)
    interface.convert2kml()



