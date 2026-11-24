import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


LINEAR_SPEED = 0.4
TURN_SPEED = 1.0

FRONT_STOP = 0.8
FRONT_CLEAR = 1.5
MAX_SIDE = 0.8

KP = 1.0
KI = 0.0
KD = 4.0

MAX_ANGULAR = LINEAR_SPEED / 0.6

IN_PLACE_SIGN = -1.0
ARC_SIGN = 1.0


class CorridorController(Node):
    def __init__(self):
        super().__init__("corridor_controller")

        self.turning = False
        self.turn_direction = 1.0

        self.integral = 0.0
        self.previous_error = 0.0

        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.process_scan,
            qos_profile_sensor_data)

        self.velocity_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10)

        self.get_logger().info("Phoenyx-I corridor controller is ready!")

    def process_scan(self, scan):
        left = self.get_sensor_distance(scan, 70, 110)
        front = self.get_sensor_distance(scan, -15, 15)
        right = self.get_sensor_distance(scan, -110, -70)

        self.get_logger().info(
            f"LEFT: {left:.2f} m | FRONT: {front:.2f} m | RIGHT: {right:.2f} m")

        if self.turning:
            if front > FRONT_CLEAR:
                self.get_logger().info("DECISION: FORWARD")
                self.turning = False
                self.integral = 0.0
                self.previous_error = 0.0
            else:
                self.publish_velocity(0.0, self.turn_direction * TURN_SPEED)
                return

        if front < FRONT_STOP:
            self.turning = True
            if left > right:
                self.turn_direction = 1.0
                self.get_logger().info("DECISION: TURN_LEFT")
            else:
                self.turn_direction = -1.0
                self.get_logger().info("DECISION: TURN_RIGHT")
            self.publish_velocity(0.0, self.turn_direction * TURN_SPEED)
            return

        error = min(left, MAX_SIDE) - min(right, MAX_SIDE)
        angular = self.pid_control(error)
        self.publish_velocity(LINEAR_SPEED, angular)

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

    def pid_control(self, error):
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error

        angular = KP * error + KI * self.integral + KD * derivative
        return max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))

    def publish_velocity(self, linear, angular):
        if linear == 0.0:
            angular *= IN_PLACE_SIGN
        else:
            angular *= ARC_SIGN
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self.velocity_publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)

    corridor_controller = CorridorController()

    try:
        rclpy.spin(corridor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        corridor_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
