import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

def euler_to_quaternion(roll, pitch, yaw):
    # Convert euler angles (roll, pitch, yaw) to quaternion
    # (assuming roll=pitch=0 for diff drive, just yaw matters)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return Quaternion(w=cy, x=0.0, y=0.0, z=sy)

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.wheel_radius = 0.05
        self.wheel_separation = 0.324

        # Pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Track last encoder readings
        self.left_encoder_old = 0.0
        self.right_encoder_old = 0.0
        self.left_encoder_new = 0.0
        self.right_encoder_new = 0.0

        # Timer to publish at e.g. 50 Hz
        self.timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(self.timer_period, self.update_odom)

        self.encoder_sub = self.create_subscription(Float32MultiArray, 'wheel_pos', self.encoder_callback, 10)

    def encoder_callback(self, msg):
        self.left_encoder_new = msg.data[0]
        self.right_encoder_new = msg.data[1]

    def update_odom(self):

        left_encoder = self.left_encoder_new
        right_encoder = self.right_encoder_new

        # 2) Compute deltas
        delta_left = left_encoder - self.left_encoder_old
        delta_right = right_encoder - self.right_encoder_old
        self.left_encoder_old = left_encoder
        self.right_encoder_old = right_encoder

        # 3) Convert to linear distance
        d_left = delta_left * self.wheel_radius
        d_right = delta_right * self.wheel_radius
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_separation

        # 4) Integrate to get (x, y, theta)
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)
        self.theta += d_theta

        # 5) Publish nav_msgs/Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y

        # Convert theta to quaternion
        q = euler_to_quaternion(0.0, 0.0, self.theta)
        odom_msg.pose.pose.orientation = q

        # Optionally fill twist if you want velocity info
        # odom_msg.twist.twist.linear.x = ...
        # odom_msg.twist.twist.angular.z = ...

        self.odom_pub.publish(odom_msg)

        # 6) Publish TF transform (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
