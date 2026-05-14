#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys, select, termios, tty

class BlueROV2Control(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        # Le topic doit correspondre EXACTEMENT au XML
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/bluerov2/controller/thruster_setpoints_sim', 
            10)
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info("Pilotage BlueROV2 v5 prêt. WSAD (Horiz), IK (Vert). CTRL+C pour quitter.")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        # On attend une touche sans bloquer
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
                
                # Commandes normalisées entre -1 et 1
                f, s, y, v = 0.0, 0.0, 0.0, 0.0
                step = 0.9

                if key == 'w': f = step
                elif key == 's': f = -step
                elif key == 'q': s = -step
                elif key == 'e': s = step
                elif key == 'a': y = -step
                elif key == 'd': y = step
                elif key == ' ': v = step
                elif key == 'x': v = -step
                elif key == 'f': f, s, y, v = 0.0, 0.0, 0.0, 0.0
                elif key == '\x03': break # CTRL+C

                # Mixage pour 8 thrusters (BlueROV2 Heavy)
                # L'ordre doit être celui du XML : 
                # 0:FrontRight, 1:FrontLeft, 2:BackRight, 3:BackLeft, 
                # 4:DiveFR, 5:DiveFL, 6:DiveBR, 7:DiveBL
                t = [0.0] * 8
                
                # Horizontal
                t[0] = f - s - y
                t[1] = f + s + y
                t[2] = -f - s + y
                t[3] = -f + s - y

                # Vertical
                t[4] = -v
                t[5] = -v
                t[6] = -v
                t[7] = -v

                # Envoi du message standardisé
                msg = Float64MultiArray()
                msg.data = [float(max(min(val, 1.0), -1.0)) for val in t]
                self.publisher_.publish(msg)

        finally:
            # Sécurité : arrêt des moteurs et restauration terminal
            stop_msg = Float64MultiArray()
            stop_msg.data = [0.0] * 8
            self.publisher_.publish(stop_msg)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    node = BlueROV2Control()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
