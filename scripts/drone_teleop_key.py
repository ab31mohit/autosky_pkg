#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped
import sys, select, os
if os.name == 'nt':
    import msvcrt, time
else:
    import tty, termios

class TeleopIrisDrone:
    def __init__(self, topic_name):
        self.MAX_CLIMB_UP_SPEED = 5           # metres per second
        self.MAX_DESCEND_DOWN_SPEED = 2.5     # metres per second
        self.MAX_LINEAR_SPEED = 20            # metres per second
        self.MAX_ANGULAR_SPEED = 2            # metres per second

        self.UP_VEL_STEP_SIZE = 0.5           # metres per second
        self.DOWN_VEL_STEP_SIZE = 0.25        # metres per second
        self.LIN_VEL_STEP_SIZE = 2            # metres per second
        self.ANG_VEL_STEP_SIZE = 0.1          # metres per second

        # mavros topic name of governing a particular drone's movement
        self.topic_name = topic_name

        # calculated lin vel that has to be fed to the drone.
        self.target_linear_vel = [0.0, 0.0, 0.0]
    
        # actual lin vel that will be fed to drone after smoothing calculated velocities.
        self.control_linear_vel = [0.0, 0.0, 0.0]

        # calculated ang vel that has to be fed to drone.
        self.target_angular_vel = 0.0

        # actual ang vel that will be fed to the drone.
        self.control_angular_vel = 0.0

        # a check condition to implement different speeds while up & down movements of the drone.
        self.moving_up = False

        # a check condition to display the control keyboards keys as we might not remember what keys will do what
        self.status = 0

        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)

        # message for telling how to control your drone.
        self.msg = """
Control Your Iris Drone!
-------------------------------------------------------------
Movement in horizontal plane: | Movement in vertical plane:
        w                     |       u
    a       d                 |   h       l
        x                     |       j

w/s : move forward/backward linearly (change linear velocity in x or front of drone)
a/d : move left/right linearly (change linear velocity in y or side of drone)

u/k : more upward/downward (change linear velocity in z or altitude of drone)
j/l : rotate counter-clock/clock (change angular velocity around z or yaw motion of drone)

space key or s : stop all movements and keep hovering at current location

CTRL-C to quit
"""

        self.error = """
        Unable to connect to Iris drone !
        """

        # initialize the node
        rospy.init_node('iris_drone_teleop')

        # define topic's publishers object
        self.pub = rospy.Publisher(self.topic_name, TwistStamped, queue_size=10)
        self.run()

    # function to get the feedback of keyboard's key when it is pressed using system events.
    def getKey(self):
        if os.name == 'nt':
            timeout = 0.1
            startTime = time.time()
            while True:
                if msvcrt.kbhit():
                    if sys.version_info[0] >= 3:
                        return msvcrt.getch().decode()
                    else:
                        return msvcrt.getch()
                elif time.time() - startTime > timeout:
                    return ''
        else:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key

    # function to display current velocities of the drone.
    def printVels(self):
        print("currently:\tlinear vel=[%s, %s, %s]\tangular vel=%s" % (
            self.target_linear_vel[0], self.target_linear_vel[1], 
            self.target_linear_vel[2], self.target_angular_vel))

    # function to convert target vel to control vel for the drone(smoothing vel transitions)
    def smooth_velocity(self, output, input, slop):
        if input > output:
            output = min(input, output + slop)
        elif input < output:
            output = max(input, output - slop)
        else:
            output = input
        return output


    def constrain(self, input, low, high):
        return max(low, min(input, high))

    def checkHorizontalLimitVelocity(self, vel):
        return self.constrain(vel, -self.MAX_LINEAR_SPEED, self.MAX_LINEAR_SPEED)

    def checkClimbUpLimitVelocity(self, vel):
        return self.constrain(vel, -self.MAX_CLIMB_UP_SPEED, self.MAX_CLIMB_UP_SPEED)

    def checkDescendDownLimitVelocity(self, vel):
        return self.constrain(vel, -self.MAX_DESCEND_DOWN_SPEED, self.MAX_DESCEND_DOWN_SPEED)

    def checkAngularLimitVelocity(self, vel):
        return self.constrain(vel, -self.MAX_ANGULAR_SPEED, self.MAX_ANGULAR_SPEED)

    # function to update drone's vel when a particular key is pressed.
    def update_velocity(self, key):

        if key == 'w':
            # Moving forward
            self.target_linear_vel[1] = self.checkHorizontalLimitVelocity(self.target_linear_vel[1] + self.LIN_VEL_STEP_SIZE)
            self.status = self.status + 1
            self.printVels()

        elif key == 'x':
            # Moving backwad
            self.target_linear_vel[1] = self.checkHorizontalLimitVelocity(self.target_linear_vel[1] - self.LIN_VEL_STEP_SIZE)
            self.status = self.status + 1
            self.printVels()

        elif key == 'd':
            # Moving rightwards
            self.target_linear_vel[0] = self.checkHorizontalLimitVelocity(self.target_linear_vel[0] + self.LIN_VEL_STEP_SIZE)
            self.status = self.status + 1
            self.printVels()

        elif key == 'a':
            # Moving leftwards
            self.target_linear_vel[0] = self.checkHorizontalLimitVelocity(self.target_linear_vel[0] - self.LIN_VEL_STEP_SIZE)
            self.status = self.status + 1
            self.printVels()

        elif key == 'u':
            # Moving up
            self.moving_up = True
            self.target_linear_vel[2] = self.checkClimbUpLimitVelocity(self.target_linear_vel[2] + self.UP_VEL_STEP_SIZE)
            self.status = self.status + 1
            self.printVels()

        elif key == 'j':
            # Moving down
            self.moving_up = False
            self.target_linear_vel[2] = self.checkDescendDownLimitVelocity(self.target_linear_vel[2] - self.DOWN_VEL_STEP_SIZE)
            self.status = self.status + 1
            self.printVels()

        elif key == 'h':
            # Rotating counter-clockwise
            self.target_angular_vel = self.checkAngularLimitVelocity(self.target_angular_vel + self.ANG_VEL_STEP_SIZE)
            self.status = self.status + 1
            self.printVels()

        elif key == 'k':
            # Rotating clockwise
            self.target_angular_vel = self.checkAngularLimitVelocity(self.target_angular_vel - self.ANG_VEL_STEP_SIZE)
            self.status = self.status + 1
            self.printVels()

        elif key == ' ' or key == 's':
            self.target_linear_vel = [0.0, 0.0, 0.0]
            self.control_linear_vel = [0.0, 0.0, 0.0]
            self.target_angular_vel = 0.0
            self.control_angular_vel = 0.0
            self.printVels()

        # print the contrl keys once again after 20 vel updates
        if self.status == 20:
            print(self.msg)
            self.status = 0

        return

    def run(self):
        # print the keys to tell how to control the drone
        print(self.msg)
        print("\n")
        self.printVels()

        try:
            while not rospy.is_shutdown():
                key = self.getKey()
                if key == '\x03':
                    break

                self.update_velocity(key)
                twist_stamped = TwistStamped()

                # calculate control vel for drone
                self.control_linear_vel[0] = self.smooth_velocity(self.control_linear_vel[0], self.target_linear_vel[0], (self.LIN_VEL_STEP_SIZE / 2.0))
                self.control_linear_vel[1] = self.smooth_velocity(self.control_linear_vel[1], self.target_linear_vel[1], (self.LIN_VEL_STEP_SIZE / 2.0))
                if self.moving_up:
                    self.control_linear_vel[2] = self.smooth_velocity(self.control_linear_vel[2], self.target_linear_vel[2], (self.UP_VEL_STEP_SIZE / 2.0))
                else:
                    self.control_linear_vel[2] = self.smooth_velocity(self.control_linear_vel[2], self.target_linear_vel[2], (self.DOWN_VEL_STEP_SIZE / 2.0))

                self.control_angular_vel = self.smooth_velocity(self.control_angular_vel, self.target_angular_vel, (self.ANG_VEL_STEP_SIZE / 2.0))

                # update topic's message object with the updated vel values
                twist_stamped.twist.linear.x = self.control_linear_vel[0]
                twist_stamped.twist.linear.y = self.control_linear_vel[1]
                twist_stamped.twist.linear.z = self.control_linear_vel[2]
                twist_stamped.twist.angular.x = 0.0
                twist_stamped.twist.angular.y = 0.0
                twist_stamped.twist.angular.z = self.control_angular_vel
                # publish the updated values to the topic
                self.pub.publish(twist_stamped)

        except Exception as e:
            print(self.error)
            print(e)

        finally:
            twist_stamped = TwistStamped()
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.linear.y = 0.0
            twist_stamped.twist.linear.z = 0.0
            twist_stamped.twist.angular.x = 0.0
            twist_stamped.twist.angular.y = 0.0
            twist_stamped.twist.angular.z = 0.0
            self.pub.publish(twist_stamped)

            if os.name != 'nt':
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)



if __name__ == "__main__":

    topic_name = 'drone1/mavros/setpoint_velocity/cmd_vel'
    TeleopIrisDrone(topic_name=topic_name)