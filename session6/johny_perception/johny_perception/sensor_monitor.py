import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SensorMonitor(Node):
    def __init__(self):
        super().__init__("sensor_monitor")

        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.process_scan,
            10 )

    def  process_scan(self, scan):
        self.get_logger().info("New /scan message received")


def main(args=None):
    rclpy.init(args=args)

    sensor_monitor = SensorMonitor()

if __name__ == "__main__":
    main()