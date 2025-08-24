import ros_robot_controller_sdk as rrc
import time

board = rrc.Board()
DC_SPEED = 1380
MID_SERVO = 64
MAX_TURN_DEGREE = 26

board.pwm_servo_set_position(0.04, [[2, DC_SPEED + 10]])
time.sleep(2)
board.pwm_servo_set_position(0.04, [[2, 1500]])
time.sleep(1)
board.pwm_servo_set_position(0.04, [[2, 1610]])
time.sleep(2)
board.pwm_servo_set_position(0.04, [[2, 1500]])
board.pwm_servo_set_position(0.04, [[1, pwm(servo_angle)]])
