#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class BlueROV2Control(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(
            Twist,
            '/motion/body_command',
            10)

        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info(
            "SimpleROV control ready. W/S=fwd/back Q/E=strafe A/D=yaw SPACE/X=up/down F=stop CTRL+C=quit"
        )

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()

                f, s, y, v = 0.0, 0.0, 0.0, 0.0
                step = 0.9

                if key == 'w':   f = step
                elif key == 's': f = -step
                elif key == 'q': s = -step
                elif key == 'e': s = step
                elif key == 'a': y = -step / 6
                elif key == 'd': y = step / 6
                elif key == ' ': v = step
                elif key == 'x': v = -step
                elif key == 'f': pass  # all zero → stop
                elif key == '\x03': break

                msg = Twist()
                msg.linear.x = f
                msg.linear.y = s
                msg.linear.z = -v  # NED: negative body Z demand moves upward
                msg.angular.z = y
                self.publisher_.publish(msg)

        finally:
            self.publisher_.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    node = BlueROV2Control()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
