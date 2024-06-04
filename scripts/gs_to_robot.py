#!/usr/bin/env python3

import rospy
import telnetlib
import json

from std_msgs.msg import Float32

class GStoRobot:
    def __init__(self):
        # Init node
        rospy.init_node("gs_to_robot")
        rospy.loginfo("\033[1;38;5;26m# GS Signal | Setting up\033[0m")
        self.host = rospy.get_param('~host_ip', "192.168.150.100")
        self.pub_topic = rospy.get_param('~pub_topic', "/scorp/signal")
        self.pub = rospy.Publisher(self.pub_topic, Float32,queue_size=10)
        
        auth = 'auth {"username":"admin","password":"admin"}'
        self.secretkey = ""

        self.signal = 0
        
        try:
            self.tn = telnetlib.Telnet(self.host,9753)
            rospy.loginfo("\033[1;38;5;26m# GS Signal | Connected to Network\033[0m")
            while self.secretkey == "":
                self.tn.write(auth.encode('ascii') + b'\n')
                output = self.tn.read_until(b'# ', timeout=1).decode('ascii')
                if output:
                    self.secretkey = output.split('"')[3]
                rospy.sleep(1)
            print(self.secretkey)
            # # Fermeture de la connexion
            # self.tn.close()
        except:
            rospy.loginfo("\033[1;38;5;208m# GS Signal| Can't connect, shutting down\033[0m")
            rospy.on_shutdown()

    def spin(self):
        rate = rospy.Rate(10)
        rospy.loginfo("\033[1;38;5;34m# GS Signal | GS Signal running\033[0m")
        info = ('systeminfo {"secretkey":"'+self.secretkey+'"}')
        while not rospy.is_shutdown():
            try:
                self.tn.write(info.encode('ascii') + b'\n')
                json_config = self.tn.read_until(b'# ', timeout=0.4).decode('ascii')
                if json_config:
                    json_config = json_config.split("console :")[0]
                    connect_list, system_info = extract_data(json_config)
                    #print_result(connect_list,system_info)

                    self.signal = float(connect_list[0]["signal"])
                    print(self.signal)
                    self.pub.publish(self.signal)

                rate.sleep()
            except:
                rospy.loginfo("\033[1;38;5;208m# GS Signal| Communication error\033[0m")
                continue
        rospy.spin()
        rospy.loginfo("\033[1;38;5;208m# GS Signal| SignalTopo shutting down\033[0m")
        self.tn.close()

def extract_data(json_string):
    data = json.loads(json_string)
    # Extraire les éléments de la liste "connectlist"
    connect_list = data["connectlist"]
    # Extraire les informations du dictionnaire "systeminfo"
    system_info = data["systeminfo"]
    return connect_list, system_info

def print_result(connect_list,system_info):
    print("Connect List:")
    for connect_info in connect_list:
        print(connect_info)
    print("\nSystem Info:")
    print(system_info)
    print("-------------")


if __name__ == '__main__':
    signal = GStoRobot()
    signal.spin()

