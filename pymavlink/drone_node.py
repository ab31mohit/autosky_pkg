#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist

def drone_node(drone_id):
    rospy.init_node(f'mission/{drone_id}', anonymous=False)
    pub_pose = rospy.Publisher(f'/{drone_id}/pose', Pose, queue_size=10)
    pub_twist = rospy.Publisher(f'/{drone_id}/twist', Twist, queue_size=10)

    def callback(data):
        try:
            index = data.name.index(drone_id)
            pose_msg = data.pose[index]
            twist_msg = data.twist[index]

            pub_pose.publish(pose_msg)
            pub_twist.publish(twist_msg)
        except ValueError:
            rospy.logwarn(f"Model '{drone_id}' not found in /gazebo/model_states")

    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    rospy.loginfo(f"{drone_id} node initialized and publishing to /{drone_id}/pose and /{drone_id}/twist\n")
    rospy.spin()

if __name__ == '__main__':
    import sys
    if len(sys.argv) != 2:
        rospy.logerr("Usage: drone_node.py <drone_id>")
        sys.exit(1)
    drone_id = sys.argv[1]
    drone_node(drone_id)
