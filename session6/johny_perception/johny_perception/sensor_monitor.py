import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SensorMonitor(Node):
    def __init__(self):
        super().__init__("sensor_monitor")

        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.process_scan,
            10)

    def process_scan(self, scan):
        left = self.get_sensor_distance(scan, 30, 90)
        front = self.get_sensor_distance(scan, -30, 30)
        right = self.get_sensor_distance(scan, -90, -30)

        self.get_logger().info(
            f"LEFT: {left:.2f} m | FRONT: {front:.2f} m | RIGHT: {right:.2f} m")

    def get_sensor_distance(self, scan, start_angle_deg, end_angle_deg):
        start_angle = math.radians(start_angle_deg)
        end_angle = math.radians(end_angle_deg)

        min_distance = math.inf 

        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment

            if angle < start_angle or angle > end_angle:
                continue

            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            min_distance = min(min_distance, distance)

        return min_distance


def main(args=None):
    rclpy.init(args=args)

    sensor_monitor = SensorMonitor()

    try:
        rclpy.spin(sensor_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()