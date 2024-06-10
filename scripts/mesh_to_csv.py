#!/usr/bin/env python3

import rospy
import json
import yaml
from datetime import datetime
import csv
import os

from std_msgs.msg import String

"""
This code convert the mesh state to a csv file so it can be easly used as a test output.
with the host tu evaluate the state of the mesh. The node needs three parameters to work propely : 
    - mesh_topic (String): The topic to subscribe to, and from whom the data will be converted.
    - output_folder (String): The path to the output folder.
    - output_name (String) : Name of the output file (t'as vraiment lu jusqu'ici pour savoir ce que pouvait être output name ?..)

"""

class MeshtoCSV:
    def __init__(self):
        # Init node
        rospy.init_node("mesh_to_csv")
        rospy.loginfo("\033[1;38;5;26m# Mesh to CSV | Setting up\033[0m")

        self.sub_topic = rospy.get_param('~sub_topic', "/mesh_state")
        self.sub = rospy.Subscriber(self.sub_topic,String,self.state_cb,queue_size=10)

        output_path = rospy.get_param('~output_folder',os.path.expanduser('~'))
        output_name = rospy.get_param('~output_name',"")+str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))+".csv"
        output_file = os.path.join(output_path, output_name)         
        self.csv = open(output_file, "w")
        self.writer = csv.writer(self.csv,delimiter=';')

        self.echo = rospy.get_param('~print', False)
        self.counter = 0

        # "host_ip;host_mac;dev_mac;expect;txbit,rxbit;multichannel;noise;ssid;signal"
        headers = ["host_ip", "host_mac", "dev_mac", "expect","txbit,rxbit", "multichannel", "noise", "ssid","signal"]
        self.writer.writerow(headers)

    def state_cb(self,msg):
        row = (msg.data.split(";"))
        self.writer.writerow(row)
        self.counter += 1

    def spin(self):
        rate = rospy.Rate(1)
        rospy.loginfo("\033[1;38;5;34m# Mesh to CSV | Running\033[0m")
        while not rospy.is_shutdown():
                if self.echo:
                    print("Row count: ",self.counter)
                rate.sleep()
        rospy.spin()
        self.csv.close()
        rospy.loginfo("\033[1;38;5;208m# Mesh to CSV| Shutting down\033[0m")

if __name__ == '__main__':
    csv = MeshtoCSV()
    csv.spin()

