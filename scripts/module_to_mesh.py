#!/usr/bin/env python3

import rospy
import telnetlib
import json
import yaml

from std_msgs.msg import String

"""
This code provide a simple mesh state publisher from an host station. It uses a telnet connection 
with the host tu evaluate the state of the mesh. The node needs the following parameters to work propely : 
    - mesh_config (.yaml): The devices list and corresponding IP and MAC addresses.
    - host_ip (192.168.150.100) : IP addresse of the main station. By default, should be the addresse 
    of the module connected to the "code".

Published Topics:
    - /mesh_state (std_msgs/String): List of states connection in the mesh

    
The node publish a String for each connection between the host module and the device in its connectlist.
The state of the connection is structured as followed : 
"host_ip;host_mac;dev_ip;dev_mac;expct;[txbit,rxbit];multichannel;noise;ssid;signal"
"""

class ModuletoMesh:
    def __init__(self):
        # Init node
        rospy.init_node("module_to_mesh")
        self.hostname = rospy.get_param('~hostname', "GS")
        rospy.loginfo("\033[1;38;5;26m# ModuletoMesh %s| Setting up\033[0m",self.hostname)
        self.cfg_file = rospy.get_param("~mesh_config",)
        self.ip, self.mac = get_info(self.cfg_file, self.hostname)
        self.pub = rospy.Publisher("/mesh_state", String ,queue_size=10)
        
        auth = 'auth {"username":"admin","password":"admin"}'
        self.secretkey = ""
        self.signal = 0
        
        try:
            self.tn = telnetlib.Telnet(self.ip,9753)
            rospy.loginfo("\033[1;38;5;26m# ModuletoMesh %s| Connected to Network\033[0m",self.hostname)
            while self.secretkey == "":
                self.tn.write(auth.encode('ascii') + b'\n')
                output = self.tn.read_until(b'# ', timeout=1).decode('ascii')
                if output:
                    self.secretkey = output.split('"')[3]
                rospy.sleep(1)
        except:
            rospy.loginfo("\033[1;38;5;208m# ModuletoMesh %s| Can't connect, shutting down\033[0m",self.hostname)
            rospy.on_shutdown()

    def connection_to_string(self,connection):
        # "host_ip;host_mac;dev_mac;expect;[txbit,rxbit];multichannel;noise;ssid;signal"
        host = (self.ip+";"+self.mac)
        dev = (connection["mac"]+";"+connection["expect"])
        rate = (connection["txbit"]+","+connection["txbit"])
        info = (connection["multichannel"]+";"+connection["noise"]+";"+connection["ssid"]+";"+connection["signal"])
        result = host+";"+dev+";"+rate+";"+info
        return result

    
    def spin(self):
        rate = rospy.Rate(10)
        rospy.loginfo("\033[1;38;5;34m# ModuletoMesh %s| Running\033[0m",self.hostname)
        info = ('systeminfo {"secretkey":"'+self.secretkey+'"}')
        while not rospy.is_shutdown():
            try:
                self.tn.write(info.encode('ascii') + b'\n')
                json_config = self.tn.read_until(b'# ', timeout=0.4).decode('ascii')
                if json_config:
                    json_config = json_config.split("console :")[0]
                    connect_list, system_info = extract_data(json_config)
                    # print_result(connect_list,system_info)

                    for con in connect_list:
                        result = self.connection_to_string(con)
                        self.pub.publish(result)

                rate.sleep()
            except:
                rospy.loginfo("\033[1;38;5;208m# ModuletoMesh %s| Communication error\033[0m",self.hostname)
                continue
        rospy.spin()
        rospy.loginfo("\033[1;38;5;208m# ModuletoMesh %s| Shutting down\033[0m",self.hostname)

        self.tn.close()


def extract_data(json_string):
    data = json.loads(json_string)
    # Extraire les éléments de la liste "connectlist"
    connect_list = data["connectlist"]
    # Extraire les informations du dictionnaire "systeminfo"
    system_info = data["systeminfo"]
    return connect_list, system_info

def get_info(cfg_file, device_name):
    with open(cfg_file) as f:
        cfg = yaml.load(f, Loader=yaml.FullLoader)
        for i in range(cfg['device_name'].__len__()):
            if cfg['device_name'][i] == device_name:
                return cfg['device_ip'][i], cfg['device_mac'][i]
            else:
                pass

def get_ip(cfg_file, device_name):
    with open(cfg_file) as f:
        cfg = yaml.load(f, Loader=yaml.FullLoader)
        for i in range(cfg['device_name'].__len__()):
            if cfg['device_name'][i] == device_name:
                return cfg['device_ip'][i], cfg['device_mac'][i]
            else:
                pass

def print_result(connect_list,system_info):
    print("Connect List:")
    for connect_info in connect_list:
        print(connect_info)
    print("\nSystem Info:")
    print(system_info)
    print("-------------")


if __name__ == '__main__':
    module = ModuletoMesh()
    module.spin()

