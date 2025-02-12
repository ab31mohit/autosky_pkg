#! /usr/bin/env python3
# Import ROS.
import rospy
# Import the API.
from autosky_pkg.py_gnc_functions import *
# To print colours (optional).
from autosky_pkg.PrintColours import *


def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller", anonymous=True)

    # Create an object for the API.
    drone = gnc_api()
    # Wait for FCU connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    drone.set_mode("GUIDED")

    # Create local reference frame.
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 3m.
    drone.takeoff(3)
    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(3)

    # Specify some waypoints
    goals = [[0, 0, 3, 0], [0, 10, 3, 0], [0, 10, 3, 180], [0, 0, 3, 180]]
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


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
