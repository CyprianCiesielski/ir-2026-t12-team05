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
OBSTACLE_CLEAR = 1.2
OPEN_SIDE = 2.0
HALF_WIDTH = 0.45
LOOKAHEAD = 1.2
MAX_SIDE = 0.8
MAX_OBSTACLE_SCANS = 8

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
        self.turn_clear = FRONT_CLEAR
        self.turn_scans = 0
        self.max_turn_scans = math.inf

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
        left, right = self.get_side_clearance(scan)
        front = self.get_front_distance(scan)

        open_left = self.get_max_distance(scan, 60, 120)
        open_right = self.get_max_distance(scan, -120, -60)

        self.get_logger().info(
            f"LEFT: {left:.2f} m | FRONT: {front:.2f} m | RIGHT: {right:.2f} m")

        if self.turning:
            self.turn_scans += 1
            if front > self.turn_clear or self.turn_scans > self.max_turn_scans:
                self.get_logger().info("DECISION: FORWARD")
                self.turning = False
                self.integral = 0.0
                self.previous_error = 0.0
            else:
                self.publish_velocity(0.0, self.turn_direction * TURN_SPEED)
                return

        if front < FRONT_STOP:
            self.turning = True
            self.turn_scans = 0

            if max(open_left, open_right) > OPEN_SIDE:
                self.turn_clear = FRONT_CLEAR
                self.max_turn_scans = math.inf
                self.get_logger().info("CORNER")
            else:
                self.turn_clear = OBSTACLE_CLEAR
                self.max_turn_scans = MAX_OBSTACLE_SCANS
                self.get_logger().info("OBSTACLE")

            if open_left > open_right:
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

    def get_side_clearance(self, scan):
        left = math.inf
        right = math.inf

        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment

            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            x = distance * math.cos(angle)
            y = distance * math.sin(angle)

            if x < -0.3 or x > LOOKAHEAD:
                continue

            if y > 0:
                left = min(left, y)
            else:
                right = min(right, -y)

        return left, right

    def get_front_distance(self, scan):
        min_distance = math.inf

        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment

            if abs(angle) > math.pi / 2:
                continue

            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            if abs(distance * math.sin(angle)) < HALF_WIDTH:
                min_distance = min(min_distance, distance * math.cos(angle))

        return min_distance

    def get_max_distance(self, scan, start_angle_deg, end_angle_deg):
        start_angle = math.radians(start_angle_deg)
        end_angle = math.radians(end_angle_deg)

        max_distance = 0.0

        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment

            if angle < start_angle or angle > end_angle:
                continue

            if distance < scan.range_min:
                continue

            max_distance = max(max_distance, distance)

        return max_distance

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
