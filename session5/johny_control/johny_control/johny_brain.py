import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

ROBOT_NAME = "johny"
TEAM_NUMBER = 5

FORWARD = "FORWARD"
TURN_LEFT = "TURN_LEFT"
TURN_RIGHT = "TURN_RIGHT"
STOP = "STOP"

LINEAR_SPEED = 0.5
ANGULAR_SPEED = 1

class RobotBrain(Node):
    def __init__(self, robot_name, team_number):
        super().__init__(f"{robot_name}_brain")

        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.process_scan,
            10 )

        self.velocity_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10 )

        self.robot_name = robot_name
        self.team_number = team_number
        self.get_logger().info(f'Team {self.team_number}: {self.robot_name} brain is ready!')

    def action_to_twist(self, action):
        command = Twist()

        if action == FORWARD:
            command.linear.x = LINEAR_SPEED
            command.angular.z = 0.0
        elif action == TURN_LEFT:
            command.linear.x = LINEAR_SPEED
            command.angular.z = 1.0
        elif action == TURN_RIGHT:
            command.linear.x = LINEAR_SPEED
            command.angular.z = -1.0
        
        return command

    def  process_scan(self, scan):
        self.scan = scan
        self.get_sensor_data()
        action = self.decide_action()
        self.get_logger().info(f"DECISION: {action}")
        command = self.action_to_twist(action)
        self.velocity_publisher.publish(command)

    def get_sensor_data(self):
        self.measurments = self.scan.ranges
        self.get_logger().info('LEFT: ' + str(self.measurments[0]) + ' m')
        self.get_logger().info('FRONT: ' + str(self.measurments[1]) + ' m')
        self.get_logger().info('RIGHT: ' + str(self.measurments[2]) + ' m')
        pass

    def decide_action(self):
        left, front, right = self.measurments
        if (front > 3.0):
            return FORWARD
        else:
            if (left < right):
                return TURN_RIGHT
            else:
                return TURN_LEFT

def main(args=None):
    rclpy.init(args=args)

    robot_brain = RobotBrain(
        ROBOT_NAME,
        TEAM_NUMBER)

    try:
        rclpy.spin(robot_brain)
    except KeyboardInterrupt:
        pass
    finally:
        robot_brain.destroy_node()
        rclpy.shutdown()
    

if __name__ == "__main__":
    main()