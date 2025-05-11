import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped
from robot_controller import ros_robot_controller_sdk as rrc
import time
import math


class DriveNode(Node):
    def __init__(self):
        super().__init__("drive_node")

        # Command subscription
        self.cmd_subscription = self.create_subscription(
            String, "/cmd_drive", self.cmd_callback, 10
        )

        # IMU subscription
        self.imu_subscription = self.create_subscription(
            Vector3Stamped, "/imu/rpy/filtered", self.imu_callback, 10
        )

        # Initialize board
        self.board = rrc.Board()

        # Control constants - matching remotecontrol.py
        self.mid = 63
        self.max_turn_degree = 30

        # Initialize servo to mid position
        self.board.pwm_servo_set_position(0.1, [[1, self.pwm(self.mid)]])
        # Arm the ESC (channel 2)
        self.board.pwm_servo_set_position(0.1, [[2, 1500]])
        time.sleep(2)
        self.get_logger().info("Ready")

        # Default values
        self.b = 1500  # Motor PWM value
        self.s = self.mid  # Servo angle

        # IMU tracking
        self.current_heading = 0.0  # Current heading from IMU in radians
        self.initial_heading = (
            None  # Will store the initial heading when 'w' is first pressed
        )
        self.target_heading = (
            None  # Will store the target heading (90 degrees right from initial)
        )
        self.using_imu_control = (
            False  # Flag to indicate if we're using IMU for steering
        )

        # PID constants (only P for now)
        self.Kp = 11.5  # Proportional gain - adjust as needed

    def pwm(self, degree):
        return round(degree * 11.1 + 500)

    def imu_callback(self, msg):
        # Store the current heading from IMU
        self.current_heading = msg.vector.z

        # If we're in IMU control mode, adjust steering based on heading error
        if self.using_imu_control and self.initial_heading is not None:
            self.steer_to_target()

    def normalize_angle(self, angle):
        """Normalize angle to be between -pi and pi"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def steer_to_target(self):
        # Calculate error (difference between current and target heading)
        error = self.normalize_angle(self.target_heading - self.current_heading)

        # Apply proportional control
        steering_adjustment = self.Kp * error

        # Limit the steering adjustment to the max turn degree
        steering_adjustment = max(
            -self.max_turn_degree, min(self.max_turn_degree, steering_adjustment)
        )

        # Set the steering angle
        self.s = self.mid + steering_adjustment

        # Update servo position
        pw = self.pwm(self.s)
        self.board.pwm_servo_set_position(0.1, [[1, pw]])

        # Log the heading control info (useful for debugging)
        self.get_logger().info(
            f"Heading: current={self.current_heading:.2f}, target={self.target_heading:.2f}, "
            f"error={error:.2f}, steering={steering_adjustment:.2f}"
        )

    def cmd_callback(self, msg):
        key = msg.data

        if key == "w":
            # Set forward speed
            self.b = 1380

            # If this is the first time 'w' is pressed or we're re-entering IMU control mode
            if not self.using_imu_control:
                # Store initial heading
                self.initial_heading = self.current_heading

                # Calculate target heading (90 degrees right = -π/2 radians from initial)
                # Note: Negative is right in radians convention
                self.target_heading = self.normalize_angle(
                    self.initial_heading - math.pi / 2
                )

                self.get_logger().info(
                    f"Starting IMU control mode. Initial heading: {self.initial_heading:.2f}, "
                    f"Target heading: {self.target_heading:.2f}"
                )

                # Enable IMU control
                self.using_imu_control = True

            # Update motor
            self.board.pwm_servo_set_position(0.1, [[2, self.b]])

        elif key == "stop" or key == " ":
            # Stop button pressed
            self.using_imu_control = False
            self.board.pwm_servo_set_position(0.1, [[2, 1500]])
            self.board.pwm_servo_set_position(0.1, [[1, self.pwm(self.mid)]])
            time.sleep(0.01)
            self.board.pwm_servo_set_position(0.1, [[1, 0]])
            time.sleep(0.01)
            self.get_logger().info("Robot stopped")

        elif key == "a":
            # Turn right - disable IMU control
            self.using_imu_control = False
            self.s = self.mid - self.max_turn_degree
            self.update_controls()

        elif key == "d":
            # Turn left - disable IMU control
            self.using_imu_control = False
            self.s = self.mid + self.max_turn_degree
            self.update_controls()

        elif key == "s":
            # Move backward - disable IMU control
            self.using_imu_control = False
            self.b = 1600
            self.update_controls()

        elif key == "x":
            # Center steering - disable IMU control
            self.using_imu_control = False
            self.s = self.mid
            self.update_controls()

    def update_controls(self):
        # Update motor and servo without IMU control
        pw = self.pwm(self.s)
        self.board.pwm_servo_set_position(0.1, [[2, self.b]])
        self.board.pwm_servo_set_position(0.1, [[1, pw]])
        time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop the robot on shutdown
        node.board.pwm_servo_set_position(0.1, [[2, 1500]])
        node.board.pwm_servo_set_position(0.1, [[1, node.pwm(node.mid)]])
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
