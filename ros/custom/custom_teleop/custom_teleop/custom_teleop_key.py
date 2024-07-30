#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
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
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
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

def checkLinearLimitVelocity(vel, model):
    if model == "burger":
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif model == "waffle" or model == "waffle_pi":
        vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel, model):
    if model == "burger":
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif model == "waffle" or model == "waffle_pi":
        vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel

class CustomTeleopKeyNode(Node):
    def __init__(self):
        super().__init__('custom_teleop_key')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.declare_parameter("model", "burger")
        self.model = self.get_parameter("model").get_parameter_value().string_value

        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0
        self.status = 0

        self.get_logger().info("Teleop Key Node has been started")

    def update_velocity(self, key):
        if key == 'w':
            self.target_linear_vel = checkLinearLimitVelocity(self.target_linear_vel + LIN_VEL_STEP_SIZE, self.model)
            self.status += 1
            print(vels(self.target_linear_vel, self.target_angular_vel))
        elif key == 'x':
            self.target_linear_vel = checkLinearLimitVelocity(self.target_linear_vel - LIN_VEL_STEP_SIZE, self.model)
            self.status += 1
            print(vels(self.target_linear_vel, self.target_angular_vel))
        elif key == 'a':
            self.target_angular_vel = checkAngularLimitVelocity(self.target_angular_vel + ANG_VEL_STEP_SIZE, self.model)
            self.status += 1
            print(vels(self.target_linear_vel, self.target_angular_vel))
        elif key == 'd':
            self.target_angular_vel = checkAngularLimitVelocity(self.target_angular_vel - ANG_VEL_STEP_SIZE, self.model)
            self.status += 1
            print(vels(self.target_linear_vel, self.target_angular_vel))
        elif key == ' ' or key == 's':
            self.target_linear_vel = 0.0
            self.control_linear_vel = 0.0
            self.target_angular_vel = 0.0
            self.control_angular_vel = 0.0
            print(vels(self.target_linear_vel, self.target_angular_vel))
        else:
            if key == '\x03':
                return False

        if self.status == 20:
            print(msg)
            self.status = 0

        twist = Twist()

        self.control_linear_vel = makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0))
        twist.linear.x = self.control_linear_vel
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        self.control_angular_vel = makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0))
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.control_angular_vel

        self.pub.publish(twist)
        return True

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin) if os.name != 'nt' else None

    rclpy.init(args=args)
    node = CustomTeleopKeyNode()

    try:
        print(msg)
        while rclpy.ok():
            key = getKey()
            if not node.update_velocity(key):
                break

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        node.pub.publish(twist)

        node.destroy_node()
        rclpy.shutdown()

        if settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
