#!/usr/bin/env python3

import rospy
import telnetlib
import json

class Station:
    def __init__(self,mac,name):
        self.mac = mac
        self.name = name
        self.signal = 0
        self.multichannel = [0,0]


class SignalTopo:
    def __init__(self):
        # Init node
        rospy.init_node("signalTopo")
        rospy.loginfo("\033[1;38;5;26m# SignalTopo | SignalTopo setting up\033[0m")
        HOST = "192.168.150.100"
        user = "admin"
        password = "admin"
        auth = 'auth {"username":"admin","password":"admin"}'
        self.secretkey = ""
        self.mac = ""
        
        try:
            self.tn = telnetlib.Telnet(HOST,9753)
            # output = self.tn.read_until(b'# ', timeout=0.1).decode('ascii')
            rospy.loginfo("\033[1;38;5;26m# SignalTopo | Connected to Network\033[0m")
            while self.secretkey == "":
                self.tn.write(auth.encode('ascii') + b'\n')
                output = self.tn.read_until(b'# ', timeout=1).decode('ascii')
                if output:
                    self.secretkey = output.split('"')[3]
            print(self.secretkey)
            # # Fermeture de la connexion
            # self.tn.close()
        except:
            rospy.loginfo("\033[1;38;5;208m# SignalTopo| Can't connect, shutting down\033[0m")
            rospy.on_shutdown()


    def spin(self):
        rate = rospy.Rate(10)
        rospy.loginfo("\033[1;38;5;34m# SignalTopo | SignalTopo running\033[0m")
        info = ('systeminfo {"secretkey":"'+self.secretkey+'"}')
        config = ('getconfig {"secretkey":"'+self.secretkey+'"}')
        while not rospy.is_shutdown():
            self.tn.write(info.encode('ascii') + b'\n')
            json_config = self.tn.read_until(b'# ', timeout=1).decode('ascii')
            if json_config:
                json_config = json_config.split("console :")[0]
                
                connect_list, system_info = extract_data(json_config)
                print("Connect List:")
                for connect_info in connect_list:
                    print(connect_info)

                print("\nSystem Info:")
                print(system_info)
                print("________________")

            rate.sleep()
        rospy.spin()
        rospy.loginfo("\033[1;38;5;208m# SignalTopo| SignalTopo shutting down\033[0m")
        self.tn.close()

def extract_data(json_string):
    data = json.loads(json_string)
    
    # Extraire les éléments de la liste "connectlist"
    connect_list = data["connectlist"]
    
    # Extraire les informations du dictionnaire "systeminfo"
    system_info = data["systeminfo"]
    
    return connect_list, system_info


if __name__ == '__main__':
    signal = SignalTopo()
    signal.spin()

