#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

import sys, select, tty, termios

LIN_VEL = 0.2
ANG_VEL = 0.5

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0.0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def move_robot(mecanum_pub, duration: int, linear_velocity: float = 0.0, angular_velocity: float = 0.0):
    twist: Twist = Twist()
    twist.linear.x = linear_velocity
    twist.angular.z = angular_velocity

    start_time = rospy.get_time()
    while rospy.get_time() - start_time < duration:
        mecanum_pub.publish(twist)
        rospy.sleep(0.1)

    twist.linear.x = 0.0
    twist.angular.z = 0.0
    mecanum_pub.publish(twist)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('mechanum_keyboard_control')
    mecanum_pub = rospy.Publisher('jetauto_controller/cmd_vel', Twist, queue_size=10)

    try:
        print("Mechanum subsystem initialized successfully")
        while not rospy.is_shutdown():
            key = get_key()
            if key == 'w':
                move_robot(mecanum_pub, 2, linear_velocity=LIN_VEL)
            elif key == 'a':
                # Rotates approx 90° to the left
                move_robot(mecanum_pub, 3, angular_velocity=ANG_VEL)
            elif key == 'd':
                # Rotates approx 90° to the right
                move_robot(mecanum_pub, 3, angular_velocity=-ANG_VEL)
            elif key == 's':
                move_robot(mecanum_pub, 2, linear_velocity=-LIN_VEL)
            elif key == '\x03':
                break
            else:
                print("Invalid key")

    except:
        print("Error in mechanum subsystem")

    finally:
        twist = Twist()
        mecanum_pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
