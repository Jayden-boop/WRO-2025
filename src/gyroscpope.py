import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
import math
import time
import ros_robot_controller_sdk as rrc
import RPi.GPIO as GPIO


board = rrc.Board()
# movement constants
MID_SERVO = 63
MAX_TURN_DEGREE = 20
DC_SPEED = 1376
PG = 0.5


def pwm(degree):  # angle must be adjusted to pwm angle for servo
    """
    Convert angle in degrees to pulse width modulation (PWM) for servo to understand

    Parameters:
    degree (int): The angle the servo should turn to.

    Returns:
    int: The equivalent pwm for the desired angle.
    """

    return round(degree * 11.1 + 500)



def radiansToDegrees(rad):
    # Convert radians to degrees in range [0, 360)
    degree = math.degrees(rad) % 360
    return degree

class GyroLoopDriver(Node):
    def __init__(self):
        super().__init__('gyro_loop_driver')

        self.subscription = self.create_subscription(
            Vector3Stamped,
            '/imu/rpy/filtered',
            self.imu_callback,
            10) #The QoS (Quality of Service) depth - this sets the size of the message queue buffer

        self.currentAngle = 0.0  # initial angle
        self.state = 'DRIVE_0'  # initial state
        self.driveStartTime = None
        self.turnStarted = False

        self.controlTimer = self.create_timer(0.01, self.controlLoop)  #controlLoop is fired every 0.01 second

    def imu_callback(self, msg):
        self.currentAngle = radiansToDegrees(msg.vector.z)

    def controlLoop(self):
        now = time.time()

        if self.state.startswith('DRIVE'):
            if self.driveStartTime is None:
                self.driveStartTime = now
                self.turnStarted = False
                self.drive(1) # speed 1
                self.steer(0.0)  # straight (to be replaced with PD controller)
                self.get_logger().info(f"Started {self.state}")
                self.get_logger().info(f"Current Angle {self.currentAngle}")
            elif now - self.driveStartTime >= 10:
                self.driveStartTime = None
                self.state = {
                    'DRIVE_0': 'TURN_90',
                    'DRIVE_90': 'TURN_180',
                    'DRIVE_180': 'TURN_270',
                    'DRIVE_270': 'TURN_0'
                }[self.state]
                self.get_logger().info(f"10 seconds elasped. Switching to {self.state}")

        elif self.state.startswith('TURN'):
            targetAngle = {
                'TURN_90': 90,
                'TURN_180': 180,
                'TURN_270': 270,
                'TURN_0': 0
            }[self.state]

            if self.turnStarted == False:
                self.drive(2) # speed 2
                self.steer(-1.0)  # counterclockwise turn (to be replaced with servo sharp turn)
                self.turnStarted = True

            if self.angleReached(targetAngle):
                self.state = {
                    'TURN_90': 'DRIVE_90',
                    'TURN_180': 'DRIVE_180',
                    'TURN_270': 'DRIVE_270',
                    'TURN_0': 'DRIVE_0'
                }[self.state]
                self.driveStartTime = None
                self.turnStarted = False
                self.get_logger().info(f"Reached {targetAngle}°, switching to {self.state}")

    def angleReached(self, target):
        diff = (self.currentAngle - target + 360) % 360
        return diff < 2 or diff > 358  # 2° tolerance

    # dummy drive function
    def drive(self, speed):
        board.pwm_servo_set_position(0.1, [[2, DC_SPEED]])
        self.get_logger().info(f"DRIVE: speed {speed}")

    # dummy steer function
    def steer(self, value: float):
        
        # value: -1.0 (left), 0.0 (center), 1.0 (right)
        #
        self.currentAngle
        if value < 0:
            self.get_logger().info("STEERING: Left") 
            servo_angle = MID_SERVO - MAX_TURN_DEGREE 
        elif value > 0:
            self.get_logger().info("STEERING: Right")
            servo_angle = MID_SERVO + MAX_TURN_DEGREE
        else:
            self.get_logger().info("STEERING: Straight")
            servo_angle = MID_SERVO
            if (self.state == 'DRIVE_0'):
                target_angle = 0
            elif (self.state == 'DRIVE_90'):
                target_angle = 90
            elif (self.state == 'DRIVE_180'):
                target_angle = 180
            elif (self.state == 'DRIVE_270'):
                target_angle = 270

            error = self.currentAngle - target_angle

        pw = pwm(servo_angle+PG*error)
        if (pw > MID_SERVO + MAX_TURN_DEGREE):
            pw = MID_SERVO + MAX_TURN_DEGREE
        if (pw < MID_SERVO - MAX_TURN_DEGREE):
            pw = MID_SERVO - MAX_TURN_DEGREE
        board.pwm_servo_set_position(0.1, [[1, pw]])

def main(args=None):
    rclpy.init(args=args)
    node = GyroLoopDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
