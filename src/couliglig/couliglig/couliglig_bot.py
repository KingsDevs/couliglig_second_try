import rclpy
from std_msgs.msg import Float32MultiArray
from controller import Robot

NODE_NAME = 'couliglig_bot_driver'
TOPIC_NAME = 'drive_cmd'

class CouligligBot:
    def init(self, webots_node, properties):
        self.__robot: Robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        self.__left_motor = self.__robot.getDevice('left_motor')
        self.__right_motor = self.__robot.getDevice('right_motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__left_motor_velocity = 0
        self.__right_motor_velocity = 0

        self.__left_sensor = self.__robot.getDevice('left_position_sensor')
        self.__right_sensor = self.__robot.getDevice('right_position_sensor')

        self.__left_sensor.enable(self.__timestep)
        self.__right_sensor.enable(self.__timestep)

        self.__imu = self.__robot.getDevice('inertial unit')
        self.__imu.enable(self.__timestep)

        rclpy.init(args=None)
        self.__node = rclpy.create_node(NODE_NAME)
        self.__node.create_subscription(Float32MultiArray, TOPIC_NAME, self.__cmd_vel_callback, 1)

        self.__motor_sensors_pub = self.__node.create_publisher(Float32MultiArray, 'wheel_pos', 10)
        self.__imu_pub = self.__node.create_publisher(Float32MultiArray, 'imu_data', 10)

    def __cmd_vel_callback(self, msg):
        self.__left_motor_velocity = msg.data[0]
        self.__right_motor_velocity = msg.data[1]

    def send_odom(self):
        motor_sensors_msg = Float32MultiArray()
        motor_sensors_msg.data = [self.__left_sensor.getValue(), self.__right_sensor.getValue()]
        self.__motor_sensors_pub.publish(motor_sensors_msg)

    def send_imu(self):
        imu_msg = Float32MultiArray()
        imu_msg.data = self.__imu.getRollPitchYaw()
        self.__imu_pub.publish(imu_msg)
        
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        self.__right_motor.setVelocity(self.__right_motor_velocity)
        self.__left_motor.setVelocity(self.__left_motor_velocity)

        self.send_odom()
        self.send_imu()

        