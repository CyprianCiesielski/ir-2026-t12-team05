from robot_arcade import (
RobotGame,
FORWARD,
TURN_LEFT,
TURN_RIGHT,
STOP,
)

ROBOT_NAME = "johny"
TEAM_NUMBER = 5

def main():
    game = RobotGame(
        robot_name=ROBOT_NAME,
        controller=decide_action,
        sensor_reporter=print_sensor_data,
        team_number=TEAM_NUMBER,
        )

    game.run()

def print_sensor_data(scan):
    measurments = scan.ranges
    print('LEFT: ' + str(measurments[0]) + ' m')
    print('FRONT: ' + str(measurments[1]) + ' m')
    print('RIGHT: ' + str(measurments[2]) + ' m')
    pass

def decide_action(scan):
    left, front, right = scan.ranges
    if (front > 3.0):
        return FORWARD
    else:
        if (left < right):
            return TURN_RIGHT
        else:
            return TURN_LEFT
   

if __name__ == "__main__":
    main()