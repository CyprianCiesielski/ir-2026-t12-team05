import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


# States
FOLLOW = "FOLLOW"
PAUSE = "PAUSE"
TURN = "TURN"

# Speeds
CRUISE_SPEED = 0.25         # m/s on straight sections
SLOW_SPEED = 0.12           # m/s when approaching a wall
TURN_SPEED_MAX = 0.6        # rad/s, rotation in place
TURN_SPEED_MIN = 0.15       # rad/s, avoids stalling near the target angle
MAX_ANGULAR_SPEED = 0.4     # rad/s while driving forward
MIN_TURN_RADIUS = 0.6       # m, osr_controller needs |v / w| > ~0.18 m

# Corridor geometry
SIDE_SECTOR_DEG = 10        # half-width of the left/right sectors around +-90 deg
FRONT_SECTOR_DEG = 15       # half-width of the front sector
SELF_FILTER = 0.25          # m, ignore returns from the rover's own body
WALL_DETECTED = 1.2         # m, a side wall closer than this is followed
DESIRED_WALL_DISTANCE = 0.75  # m, half of the corridor width
FRONT_SLOW_DOWN = 1.5       # m, start slowing down
FRONT_STOP = 0.7            # m, front wall distance that triggers a turn
SIDE_OPEN = 1.5             # m, a side farther than this is an open path

# PID on the lateral (centring) error
KP = 1.5
KI = 0.02
KD = 0.5
INTEGRAL_LIMIT = 1.0
# Heading hold (IMU) keeps the rover parallel to the corridor
K_HEADING = 0.5

TURN_TOLERANCE = math.radians(3)
TURN_GAIN = 1.5
PAUSE_TIME = 0.6            # s, lets the servos reach their angles


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


class CorridorController(Node):
    def __init__(self):
        super().__init__("corridor_controller")

        self.left = math.inf
        self.front = math.inf
        self.right = math.inf

        self.yaw = None
        self.target_yaw = None

        self.state = FOLLOW
        self.next_state = FOLLOW
        self.pause_until = 0.0

        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.process_scan,
            qos_profile_sensor_data)

        self.imu_subscription = self.create_subscription(
            Imu,
            "/imu_plugin/out",
            self.process_imu,
            qos_profile_sensor_data)

        self.velocity_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10)

        self.get_logger().info("Phoenyx-I corridor controller is ready!")

    def process_imu(self, imu):
        q = imu.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # The corridor direction at start-up is the first reference heading.
        if self.target_yaw is None:
            self.target_yaw = self.yaw

    def process_scan(self, scan):
        self.left = self.get_sensor_distance(
            scan, 90 - SIDE_SECTOR_DEG, 90 + SIDE_SECTOR_DEG)
        self.front = self.get_sensor_distance(
            scan, -FRONT_SECTOR_DEG, FRONT_SECTOR_DEG)
        self.right = self.get_sensor_distance(
            scan, -90 - SIDE_SECTOR_DEG, -90 + SIDE_SECTOR_DEG)

        self.get_logger().info(
            f"[{self.state}] LEFT: {self.left:.2f} m | "
            f"FRONT: {self.front:.2f} m | RIGHT: {self.right:.2f} m",
            throttle_duration_sec=1.0)

        if self.yaw is None:
            self.get_logger().warn(
                "Waiting for IMU data on /imu_plugin/out...",
                throttle_duration_sec=2.0)
            return

        now = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9

        if self.state == FOLLOW:
            self.follow_corridor(now)
        elif self.state == PAUSE:
            self.publish_velocity(0.0, 0.0)
            if now >= self.pause_until:
                self.change_state(self.next_state)
        elif self.state == TURN:
            self.turn(now)

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
            if distance < max(scan.range_min, SELF_FILTER):
                continue
            if distance > scan.range_max:
                continue

            min_distance = min(min_distance, distance)

        return min_distance

    def follow_corridor(self, now):
        if self.front < FRONT_STOP:
            self.choose_direction(now)
            return

        if self.front < FRONT_SLOW_DOWN:
            linear = SLOW_SPEED
        else:
            linear = CRUISE_SPEED

        angular = self.pid_control(now)
        max_angular = min(MAX_ANGULAR_SPEED, linear / MIN_TURN_RADIUS)
        angular = clamp(angular, -max_angular, max_angular)

        self.publish_velocity(linear, angular)

    def choose_direction(self, now):
        left_open = self.left > SIDE_OPEN
        right_open = self.right > SIDE_OPEN

        if left_open and (not right_open or self.left >= self.right):
            turn_angle = math.pi / 2
            decision = "TURN_LEFT"
        elif right_open:
            turn_angle = -math.pi / 2
            decision = "TURN_RIGHT"
        else:
            turn_angle = math.pi
            decision = "TURN_AROUND (dead end)"

        self.get_logger().info(
            f"Corner detected -> {decision} "
            f"(L: {self.left:.2f} m, F: {self.front:.2f} m, "
            f"R: {self.right:.2f} m)")

        self.target_yaw = normalize_angle(self.target_yaw + turn_angle)
        self.start_pause(now, TURN)

    def turn(self, now):
        error = normalize_angle(self.target_yaw - self.yaw)

        if abs(error) < TURN_TOLERANCE:
            self.get_logger().info("Turn completed")
            self.reset_pid()
            self.start_pause(now, FOLLOW)
            return

        angular = clamp(TURN_GAIN * abs(error), TURN_SPEED_MIN, TURN_SPEED_MAX)
        self.publish_velocity(0.0, math.copysign(angular, error))

    def start_pause(self, now, next_state):
        self.pause_until = now + PAUSE_TIME
        self.next_state = next_state
        self.change_state(PAUSE)

    def change_state(self, state):
        if state != self.state:
            self.get_logger().info(f"State: {self.state} -> {state}")
        self.state = state

    def pid_control(self, now):
        left_wall = self.left < WALL_DETECTED
        right_wall = self.right < WALL_DETECTED

        # Positive error: the rover is too close to the right wall.
        if left_wall and right_wall:
            error = (self.left - self.right) / 2.0
        elif left_wall:
            error = self.left - DESIRED_WALL_DISTANCE
        elif right_wall:
            error = DESIRED_WALL_DISTANCE - self.right
        else:
            error = 0.0

        if self.previous_time is None:
            dt = 0.0
        else:
            dt = now - self.previous_time

        derivative = 0.0
        if dt > 0.0:
            self.integral = clamp(
                self.integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
            if self.previous_error is not None:
                derivative = (error - self.previous_error) / dt

        self.previous_error = error
        self.previous_time = now

        heading_error = normalize_angle(self.target_yaw - self.yaw)

        return (KP * error + KI * self.integral + KD * derivative
                + K_HEADING * heading_error)

    def reset_pid(self):
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

    def publish_velocity(self, linear, angular):
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
