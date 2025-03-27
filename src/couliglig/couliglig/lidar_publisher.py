from sensor_msgs.msg import LaserScan
from rclpy.node import Node

class LidarScanPublisher(Node):
    def __init__(self, robot, timestep):
        super().__init__('lidar_scan_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.lidar = robot.getDevice('lidar')
        self.lidar.enable(timestep)
        self.lidar.enablePointCloud()
        self.timer = self.create_timer(timestep / 1000.0, self.publish_scan)

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar'

        # Webots Lidar parameters
        resolution = self.lidar.getHorizontalResolution()
        fov = self.lidar.getFov()
        max_range = self.lidar.getMaxRange()
        min_range = 0.1

        msg.angle_min = -fov / 2.0
        msg.angle_max = fov / 2.0
        msg.angle_increment = fov / resolution
        msg.range_min = min_range
        msg.range_max = max_range
        msg.scan_time = 0.1

        ranges = self.lidar.getRangeImage()
        msg.ranges = list(ranges)

        self.publisher_.publish(msg)
