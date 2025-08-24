# ~/ros2_ws/src/my_robot_controller/my_robot_controller/motor_driver_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

# Your hardware-specific imports
import ros_robot_controller_sdk as rrc

# import RPi.GPIO as GPIO # The SDK probably handles this


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node")

        # --- Hardware Setup ---
        try:
            self.board = rrc.Board()
            self.get_logger().info("rrc.Board initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize rrc.Board: {e}")
            # You might want to shut down if the hardware isn't found
            rclpy.shutdown()
            return

        # --- Constants from your original script ---
        self.MID_SERVO = 65
        self.MAX_TURN_DEGREE = 26

        # --- ROS2 Communications ---
        self.subscription = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        self.get_logger().info("Motor Driver Node has been started.")

    def pwm(self, degree):
        """Your original pwm conversion function."""
        return round(degree * 11.1 + 500)

    def cmd_vel_callback(self, msg):
        """Listen to Twist commands and translates them to PWM signals."""
        # msg.linear.x is forward speed in m/s
        # msg.angular.z is turning speed in rad/s

        # --- Translate Angular Velocity to Servo Degree ---
        # This is the reverse of the mapping in the controller node.
        max_rad_s = 0.5  # Must match the controller!
        normalized_turn = -msg.angular.z / max_rad_s  # The sign might need flipping

        # Clamp the value between -1 and 1
        normalized_turn = max(-1.0, min(1.0, normalized_turn))

        servo_degree = self.MID_SERVO + normalized_turn * self.MAX_TURN_DEGREE

        # --- Translate Linear Velocity to DC Motor PWM ---
        # This requires calibration. Let's create a simple mapping.
        # 1500 = stop, 1380 = forward, 1620 = reverse
        dc_pwm = 1500  # Default to stop
        if msg.linear.x > 0.01:  # Forward motion
            # Map [0.01, 0.2] m/s to [1480, 1380] pwm
            dc_pwm = 1480 - (msg.linear.x * 500)  # Simple linear map, CALIBRATE THIS!
        elif msg.linear.x < -0.01:  # Reverse motion
            # Map [-0.01, -0.2] m/s to [1520, 1620] pwm
            dc_pwm = 1520 - (msg.linear.x * 500)  # CALIBRATE THIS!

        dc_pwm = int(max(1380, min(1620, dc_pwm)))  # Clamp to safe values

        # --- Send Commands to Hardware ---
        try:
            servo_pwm_val = self.pwm(servo_degree)
            self.board.pwm_servo_set_position(
                0.1, [[1, servo_pwm_val]]
            )  # Servo on channel 1
            self.board.pwm_servo_set_position(
                0.1, [[2, dc_pwm]]
            )  # DC motor on channel 2
        except Exception as e:
            self.get_logger().error(f"Failed to send command to board: {e}")


def main(args=None):
    rclpy.init(args=args)
    motor_driver_node = MotorDriverNode()
    rclpy.spin(motor_driver_node)

    # Stop the robot when the node is shut down
    motor_driver_node.board.pwm_servo_set_position(0.1, [[2, 1500]])
    motor_driver_node.board.pwm_servo_set_position(
        0.1, [[1, motor_driver_node.pwm(motor_driver_node.MID_SERVO)]]
    )

    motor_driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
