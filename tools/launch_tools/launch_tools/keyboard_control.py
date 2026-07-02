#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys, select, termios, tty

class BlueROV2Control(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/bluerov2/controller/thruster_setpoints_sim',
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

                # Mixer for 6 thrusters on simple_rov.scn
                # Order matches XML: 0:FrontRight 1:FrontLeft 2:BackRight 3:BackLeft
                #                    4:VertFront  5:VertBack
                t = [0.0] * 6

                # Horizontal (diagonal 45-deg layout)
                t[0] =  f - s - y   # FrontRight
                t[1] =  f + s + y   # FrontLeft
                t[2] = -f - s + y   # BackRight
                t[3] = -f + s - y   # BackLeft

                # Vertical (forward = down in NED → negate for surface)
                t[4] = -v           # VertFront
                t[5] = -v           # VertBack

                msg = Float64MultiArray()
                msg.data = [float(max(min(val, 1.0), -1.0)) for val in t]
                self.publisher_.publish(msg)

        finally:
            stop_msg = Float64MultiArray()
            stop_msg.data = [0.0] * 6
            self.publisher_.publish(stop_msg)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    node = BlueROV2Control()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
