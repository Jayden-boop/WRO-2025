import time
import ros_robot_controller_sdk as rrc


board = rrc.Board()

# arm the ESC
board.pwm_servo_set_position(0.1, [[2, 1500]])
time.sleep(6)

while True:
    board.pwm_servo_set_position(0.1, [[2, int(input("Enter the speed: "))]])
    time.sleep(2)
