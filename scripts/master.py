#! /usr/bin/env python

# Import ROS.
import rospy
# Import the API.
from autosky_pkg.py_gnc_functions import *
# To print colours (optional).
from autosky_pkg.PrintColours import *


def main():

    # Iniatialize Master Node
    rospy.init_node('Mission_Master', anonymous=False)

    print("Master Node is running")

    # Number of drones (constant)
    NumDrones = 2    

    # Create an object for the API.
    drone = gnc_api()
    
    # Wait for FCU connection.
    drone.wait4connect()

    # Wait for the mode to be switched.
    drone.set_mode("GUIDED")

    # Create local reference frame.
    drone.initialize_local_frame()

    # Request takeoff to an altitude.
    altitude = 5
    drone.takeoff(altitude)

    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(3)

    # Specify some waypoints
    goals = [[0, 0, altitude, 0], [0, 10, altitude, 0], [0, 10, altitude, 180], [0, 0, altitude, 180], [0, 0, altitude, 0]]
    i = 0

    while i < len(goals):
        drone.set_destination(
            x=goals[i][0], y=goals[i][1], z=goals[i][2], psi=goals[i][3])
        rate.sleep()
        if drone.check_waypoint_reached():
            i += 1
   
    rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)

    # Land after all waypoints is reached.
    drone.land()

    # if not rospy.is_shutdown():
    #     rospy.spin()

    
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()





