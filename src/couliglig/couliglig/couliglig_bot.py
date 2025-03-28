import rclpy
import math
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from controller import Robot, PositionSensor, InertialUnit, Lidar, Motor

NODE_NAME = 'couliglig_bot_driver'
TOPIC_NAME = 'drive_cmd'

WHEEL_RADIUS = 0.05
WHEEL_SEPARATION = 0.324

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

class CouligligBot:
    def init(self, webots_node, properties):
        self.__robot: Robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        self.__left_motor: Motor = self.__robot.getDevice('left_motor')
        self.__right_motor: Motor = self.__robot.getDevice('right_motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__left_motor_velocity = 0
        self.__right_motor_velocity = 0

        self.__left_sensor: PositionSensor = self.__robot.getDevice('left_position_sensor')
        self.__right_sensor: PositionSensor = self.__robot.getDevice('right_position_sensor')

        self.__left_sensor.enable(self.__timestep)
        self.__right_sensor.enable(self.__timestep)

        self.__x = 0.0
        self.__y = 0.0
        self.__theta = 0.0

        self.__left_encoder_old = 0.0
        self.__right_encoder_old = 0.0

        self.__imu: InertialUnit = self.__robot.getDevice('inertial unit')
        self.__imu.enable(self.__timestep)

        # self.__lidar: Lidar = self.__robot.getDevice('LDS-01')
        # self.__lidar.enable(self.__timestep)

        rclpy.init(args=None)
        self.__node = rclpy.create_node(NODE_NAME)
        self.__node.create_subscription(Float32MultiArray, TOPIC_NAME, self.__cmd_vel_callback, 1)

        self.__tf_broadcaster = TransformBroadcaster(self.__node)

        self.__odom_pub = self.__node.create_publisher(Odometry, 'odom', 10)
        self.__imu_pub = self.__node.create_publisher(Imu, 'imu/data', 10)
        # self.__lidar_pub = self.__node.create_publisher(LaserScan, 'scan', 10)

        self.__timer_period = 0.02 # 50 Hz
        self.__timer_odom = self.__node.create_timer(self.__timer_period, self.send_odom)
        # self.__timer_lidar = self.__node.create_timer(self.__timer_period, self.send_laserscan)
        # self.__timer_imu = self.__node.create_timer(self.__timer_period, self.send_imu)

    def __cmd_vel_callback(self, msg):
        self.__left_motor_velocity = msg.data[0]
        self.__right_motor_velocity = msg.data[1]

    def send_odom(self):
        left_encoder = self.__left_sensor.getValue()
        right_encoder = self.__right_sensor.getValue()

        # 2) Compute deltas
        delta_left = left_encoder - self.__left_encoder_old
        delta_right = right_encoder - self.__right_encoder_old
        self.__left_encoder_old = left_encoder
        self.__right_encoder_old = right_encoder

        # 3) Convert to linear distance
        d_left = delta_left * WHEEL_RADIUS
        d_right = delta_right * WHEEL_RADIUS
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / WHEEL_SEPARATION

        # 4) Integrate to get (x, y, theta)
        self.__x += d_center * math.cos(self.__theta)
        self.__y += d_center * math.sin(self.__theta)
        self.__theta += d_theta

        # 5) Publish nav_msgs/Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.__node.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.__x
        odom_msg.pose.pose.position.y = self.__y

        # Convert theta to quaternion
        q = euler_to_quaternion(0.0, 0.0, self.__theta)
        odom_msg.pose.pose.orientation = q

        # Optionally fill twist if you want velocity info
        # odom_msg.twist.twist.linear.x = ...
        # odom_msg.twist.twist.angular.z = ...

        self.__odom_pub.publish(odom_msg)

        # 6) Publish TF transform (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.__x
        t.transform.translation.y = self.__y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.__tf_broadcaster.sendTransform(t)

    def send_imu(self):
        imu_data = self.__imu.getRollPitchYaw()

        imu_msg = Imu()
        imu_msg.header.stamp = self.__node.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.orientation = euler_to_quaternion(imu_data[0], imu_data[1], imu_data[2])

        self.__imu_pub.publish(imu_msg)

    def send_laserscan(self):
        msg = LaserScan()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.header.frame_id = 'LDS-01'

        resolution = self.__lidar.getHorizontalResolution()
        fov = self.__lidar.getFov()
        max_range = self.__lidar.getMaxRange()
        min_range = 0.1

        msg.angle_min = -fov / 2.0
        msg.angle_max = fov / 2.0
        msg.angle_increment = fov / resolution
        msg.range_min = min_range
        msg.range_max = max_range
        msg.scan_time = self.__timestep / 1000.0

        msg.ranges = list(self.__lidar.getRangeImage())
        self.__lidar_pub.publish(msg)

        
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        self.__right_motor.setVelocity(self.__right_motor_velocity)
        self.__left_motor.setVelocity(self.__left_motor_velocity)

        # self.send_odom()
        # self.send_imu()
        # self.send_laserscan()

        