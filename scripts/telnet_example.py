#!/usr/bin/env python3

import rospy
import telnetlib
import json

class TelnetExample:
    def __init__(self):
        # Init node
        rospy.init_node("signalTopo")
        rospy.loginfo("\033[1;38;5;26m# TelnetExample | TelnetExample setting up\033[0m")
        self.host = rospy.get_param('~host_ip', "192.168.150.100")
        auth = 'auth {"username":"admin","password":"admin"}'
        self.secretkey = ""
        self.mac = ""
        
        try:
            self.tn = telnetlib.Telnet(self.host,9753)
            rospy.loginfo("\033[1;38;5;26m# TelnetExample | Connected to mesh\033[0m")
            while self.secretkey == "":
                self.tn.write(auth.encode('ascii') + b'\n')
                output = self.tn.read_until(b'# ', timeout=0.5).decode('ascii')
                if output:
                    self.secretkey = output.split('"')[3]
            # print(self.secretkey)
            # Fermeture de la connexion
            # self.tn.close()
        except:
            rospy.loginfo("\033[1;38;5;208m# TelnetExample| Can't connect, shutting down\033[0m")
            rospy.on_shutdown()


    def spin(self):
        rate = rospy.Rate(10)
        rospy.loginfo("\033[1;38;5;34m# TelnetExample | Running\033[0m")
        info = ('systeminfo {"secretkey":"'+self.secretkey+'"}')
        config = ('getconfig {"secretkey":"'+self.secretkey+'"}')
        while not rospy.is_shutdown():
            self.tn.write(info.encode('ascii') + b'\n')
            json_config = self.tn.read_until(b'# ', timeout=1).decode('ascii')
            if json_config:
                json_config = json_config.split("console :")[0]
                
                connect_list, system_info = extract_data(json_config)
                print_result(connect_list,system_info)

            rate.sleep()
        rospy.spin()
        rospy.loginfo("\033[1;38;5;208m# TelnetExample| Shutting down\033[0m")
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
    telnet_example = TelnetExample()
    telnet_example.spin()

