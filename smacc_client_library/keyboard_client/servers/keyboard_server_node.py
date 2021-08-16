#!/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16

import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)


def getKey():
    global settings
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__("keyboard_node")
        self.pub = self.create_publisher(UInt16, "keyboard_unicode", 1)
        timer_period = 0.20  # seconds
        self.timer = self.create_timer(timer_period, self.timer_update)
        self.i = 0

    def timer_update(self):
        try:
            key = getKey()
            # rospy.loginfo(type(key))
            msg = UInt16()
            msg.data = ord(key)

            self.pub.publish(msg)

        except Exception as e:
            print(e)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)

    node = KeyboardPublisher()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# ---------------------------------

# import std_msgs
#


# class KeyboardPublisher(Node):
#     def __init__(self):
#         super().__init__('keyboard_node')
#         self.pub = self.create_publisher(UInt16, 'keyboard_unicode', 1)
#         timer_period = 0.20  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_update)
#         self.i = 0

#     def timer_update(self):
#         try:
#             key = getKey()
#             #rospy.loginfo(type(key))
#             msg = UInt16()
#             msg.data = ord(key)

#             self.pub.publish(msg)

#         except Exception as e:
#             print(e)

#         finally:
#             termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

# def main(args=None):
#     rclpy.init(args=args)
#     node = KeyboardPublisher()

#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
