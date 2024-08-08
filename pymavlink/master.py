import rospy
from geometry_msgs.msg import Vector3, Pose
from gazebo_msgs.msg import ModelStates
import json
import os
from droneClass import DRONE
from multiprocessing import Process


# Iniatialize Master Node
rospy.init_node('/mission/master', anonymous=False)

NumDrones = 2           # Number of drones (constant)
drones = DRONE(NumDrones)
drones.createDroneNodes()

if not rospy.is_shutdown():

    print("Master Node is running")
rospy.spin()

