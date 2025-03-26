import rclpy
from std_msgs.msg import Float32MultiArray
from controller import Robot

NODE_NAME = 'couliglig_bot_driver'
TOPIC_NAME = 'drive_cmd'

class CouligligBot:
    def init(self, webots_node, properties):
        self.__robot: Robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left_motor')
        self.__right_motor = self.__robot.getDevice('right_motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__left_motor_velocity = 0
        self.__right_motor_velocity = 0

        self.__left_sensor = self.__robot.getDevice('left_motor_sensor')
        self.__right_sensor = self.__robot.getDevice('right_motor_sensor')

        self.__left_sensor.enable(self.__robot.getBasicTimeStep())
        self.__right_sensor.enable(self.__robot.getBasicTimeStep())

        rclpy.init(args=None)
        self.__node = rclpy.create_node(NODE_NAME)
        self.__node.create_subscription(Float32MultiArray, TOPIC_NAME, self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, msg):
        self.__left_motor_velocity = msg.data[0]
        self.__right_motor_velocity = msg.data[1]
        
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        self.__right_motor.setVelocity(self.__right_motor_velocity)
        self.__left_motor.setVelocity(self.__left_motor_velocity)

        