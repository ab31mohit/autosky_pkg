#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped
import sys, select, os
if os.name == 'nt':
    import msvcrt, time
else:
    import tty, termios

# speed limits for motion in vertical plane
MAX_CLIMB_UP_SPEED = 5           # metres per second
MAX_DESCEND_DOWN_SPEED = 2.5     # metres per second

# speed limits for motion in horizontal plane
MAX_LINEAR_SPEED = 20            # metres per second
MAX_ANGULAR_SPEED = 2            # metres per second

# Speed increments/decrements steps
UP_VEL_STEP_SIZE = 0.5           # metres per second
DOWN_VEL_STEP_SIZE = 0.25        # metres per second

LIN_VEL_STEP_SIZE = 2            # metres per second
ANG_VEL_STEP_SIZE = 0.1          # metres per second

msg = """
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

e = """
Unable to connect to Iris drone !
"""

def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''
    
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel=[%s, %s, %s]\tangular vel=%s" % (target_linear_vel[0], target_linear_vel[1], target_linear_vel[2], target_angular_vel)


def smooth_velocity(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input

    return input

def checkHorizontalLimitVelocity(vel):
    vel = constrain(vel, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
    return vel

def checkClimbUpLimitVelocity(vel):
    vel = constrain(vel, -MAX_CLIMB_UP_SPEED, MAX_CLIMB_UP_SPEED)
    return vel

def checkDescendDownLimitVelocity(vel):
    vel = constrain(vel, -MAX_DESCEND_DOWN_SPEED, MAX_DESCEND_DOWN_SPEED)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
    return vel

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('iris_drone_teleop')
    topic_name = 'drone1/mavros/setpoint_velocity/cmd_vel'
    pub = rospy.Publisher(topic_name, TwistStamped, queue_size=10)

    status = 0
    target_linear_vel = [0.0, 0.0, 0.0]
    control_linear_vel = [0.0, 0.0, 0.0]
    target_angular_vel = 0.0     # rotating drone counterclock or clockwise in horizontal plane
    control_angular_vel = 0.0
    moving_up = False

    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            if key == 'w':
                # Moving forward
                target_linear_vel[1] = checkHorizontalLimitVelocity(target_linear_vel[1] + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))

            elif key == 'x':
                # Moving backwad
                target_linear_vel[1] = checkHorizontalLimitVelocity(target_linear_vel[1] - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))

            elif key == 'd':
                # Moving rightwards
                target_linear_vel[0] = checkHorizontalLimitVelocity(target_linear_vel[0] + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))

            elif key == 'a':
                # Moving leftwards
                target_linear_vel[0] = checkHorizontalLimitVelocity(target_linear_vel[0] - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))

            elif key == 'u':
                # Moving up
                moving_up = True
                target_linear_vel[2] = checkClimbUpLimitVelocity(target_linear_vel[2] + UP_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))

            elif key == 'j':
                # Moving down
                moving_up = False
                target_linear_vel[2] = checkDescendDownLimitVelocity(target_linear_vel[2] - DOWN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))

            elif key == 'h':
                # Rotating counter-clockwise
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))

            elif key == 'k':
                # Rotating clockwise
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))

            elif key == ' ' or key == 's':
                target_linear_vel = [0.0, 0.0, 0.0]
                control_linear_vel = [0.0, 0.0, 0.0]
                target_angular_vel = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))

            else:
                if key == '\x03':
                    break

            if status == 20:
                print(msg)
                status = 0

            twist_stamped = TwistStamped()

            control_linear_vel[0] = smooth_velocity(control_linear_vel[0], target_linear_vel[0], (LIN_VEL_STEP_SIZE/2.0))
            control_linear_vel[1] = smooth_velocity(control_linear_vel[1], target_linear_vel[1], (LIN_VEL_STEP_SIZE/2.0))
            if moving_up:
                control_linear_vel[2] = smooth_velocity(control_linear_vel[2], target_linear_vel[2], (UP_VEL_STEP_SIZE/2.0))
            else:
                control_linear_vel[2] = smooth_velocity(control_linear_vel[2], target_linear_vel[2], (DOWN_VEL_STEP_SIZE/2.0))

            control_angular_vel = smooth_velocity(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))

            twist_stamped.twist.linear.x = control_linear_vel[0]
            twist_stamped.twist.linear.y = control_linear_vel[1]
            twist_stamped.twist.linear.z = control_linear_vel[2]
            twist_stamped.twist.angular.x = 0.0
            twist_stamped.twist.angular.y = 0.0
            twist_stamped.twist.angular.z = control_angular_vel
            pub.publish(twist_stamped)

    except Exception as ex:
        print(e)
        print(str(ex))

    finally:
        twist_stamped = TwistStamped()
        twist_stamped.twist.linear.x = 0.0
        twist_stamped.twist.linear.y = 0.0
        twist_stamped.twist.linear.z = 0.0
        twist_stamped.twist.angular.x = 0.0
        twist_stamped.twist.angular.y = 0.0
        twist_stamped.twist.angular.z = 0.0
        pub.publish(twist_stamped)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


