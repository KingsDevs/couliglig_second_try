from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node
from controller import Robot
import math

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        self.__imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.__imu_sub_data = self.create_subscription(Float32MultiArray, 'imu_data', self.__imu_callback, 10)

        self.__imu_data = [0.0, 0.0, 0.0]

        self.__timer_period = 0.02  # 50 Hz
        self.__timer = self.create_timer(self.__timer_period, self.__publish_imu)
    
    def __imu_callback(self, msg):
        self.__imu_data = msg.data

    def __publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.orientation = euler_to_quaternion(self.__imu_data[0], self.__imu_data[1], self.__imu_data[2])

        self.__imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    