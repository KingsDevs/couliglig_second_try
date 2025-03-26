import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import sys
import select
import termios
import tty
import numpy as np

MAX_SPEED = 3.0

class DriveCommandPublisher(Node):
    def __init__(self):
        super().__init__('drive_command_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'drive_cmd', 10)
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
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
                key = self.getKey()
                msg = Float32MultiArray()

                if key == 'w':
                    msg.data = np.array([MAX_SPEED, MAX_SPEED], dtype=np.float32).tolist()
                elif key == 's':
                    msg.data = np.array([-MAX_SPEED, -MAX_SPEED], dtype=np.float32).tolist()
                elif key == 'a':
                    msg.data = np.array([-MAX_SPEED, MAX_SPEED], dtype=np.float32).tolist()
                elif key == 'd':
                    msg.data = np.array([MAX_SPEED, -MAX_SPEED], dtype=np.float32).tolist()
                elif key == 'q':
                    break
                else:
                    msg.data = np.array([0.0, 0.0], dtype=np.float32).tolist()

                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: {key}' + str(msg.data))

        except Exception as e:
            self.get_logger().info('Error: ' + str(e))
            self.get_logger().info('Exiting...')

def main(args=None):
    rclpy.init(args=args)
    node = DriveCommandPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()