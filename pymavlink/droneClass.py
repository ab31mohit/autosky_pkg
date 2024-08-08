from pymavlink import mavutil
import time
from geometry_msgs.msg import Vector3, Pose, Twist
from gazebo_msgs.msg import ModelStates
import rospy
# import numpy as np
# import cv2
# import json
import os
from multiprocessing import Process

class DRONE:

    def __init__(self, NumDrones):
        self.nod = NumDrones
        self.drone_connections = [None] * self.nod
        self.udpin = [14551 + 10 * i for i in range(self.nod)]
        self.pkg_name = 'autosky_test_pkg'


    ####################################################################
    # Part 1 : functions for creating separate nodes
    def create_drone_process(self, drone_id):
        os.system(f'rosrun {self.pkg_name} drone_node.py {drone_id}')

    def createDroneNodes(self):
        processes = []
        for i in range(self.nod):
            drone_id = f"drone{i+1}"
            p = Process(target=self.create_drone_process, args=(drone_id,))
            p.start()
            processes.append(p)

        for p in processes:
            p.join()
            
    
    #######################################################################
    # Part 2 : functions for commanding drones    

    def wait4connect(self):              # This function creates connection to all the drones separately using udpin            
        for i in range(self.nod):
            # Start a connection listening to a UDP port
            self.drone_connections[i] = mavutil.mavlink_connection(f'udpin:localhost:{self.udpin[i]}') 

            # This sets the system and component ID of remote system for the link
            self.drone_connections[i].wait_heartbeat()
            print(f"drone {i}:\n")
            print("Heartbeat from system (system %u component %u)" %
                    (self.drone_connections[i].target_system, self.drone_connections[i].target_component))
            
            print("\n\n")
            
        return
    
    
    def armMotors(the_connection):
        
        print('\nArming\n:')
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)
        print("\n")

    def set_modeGuided(the_connection):
        # set mode guided
        print('\nSetting Mode Guided:\n')
        mode = 'GUIDED'
        mode_id = the_connection.mode_mapping()[mode]
        the_connection.mav.set_mode_send(the_connection.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)

        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)
        print("\n")

