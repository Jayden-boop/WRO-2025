import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped
from robot_controller import ros_robot_controller_sdk as rrc
import time
import math
import threading


class RectangularPathNode(Node):
    def __init__(self):
        super().__init__("rectangular_path_node")

        self.imu_subscription = self.create_subscription(
            Vector3Stamped, "/imu/rpy/filtered", self.imu_callback, 10
        )

        self.board = rrc.Board()

        self.mid = 56.5
        self.max_turn_degree = 30

        self.board.pwm_servo_set_position(0.1, [[1, self.pwm(self.mid)]])
        self.board.pwm_servo_set_position(0.1, [[2, 1500]])
        time.sleep(2)
        self.get_logger().info("Ready")

        self.b = 1500
        self.s = self.mid

        self.current_heading = 0.0
        self.target_heading = None
        self.using_imu_control = False

        self.Kp = 11.5
        self.Kd = 0.5
        self.prev_error = 0.0
        self.last_error_time = None

        self.state = "IDLE"
        self.state_start_time = None
        self.straight_drive_duration = 3.0
        self.completed_turns = 0
        self.target_turns = 4
        self.running = True

        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.timer = self.create_timer(0.05, self.state_machine)

    def pwm(self, degree):
        return round(degree * 11.1 + 500)

    def input_loop(self):
        while self.running:
            key = input("Press 'x' to stop: ")
            if key.lower() == "x":
                self.get_logger().info("Manual stop requested")
                self.stop_robot()
                self.running = False
                break

    def stop_robot(self):
        self.state = "COMPLETE"
        self.b = 1500
        self.board.pwm_servo_set_position(0.1, [[2, self.b]])
        self.board.pwm_servo_set_position(0.1, [[1, self.pwm(self.mid)]])
        self.get_logger().info("Robot stopped")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def imu_callback(self, msg):
        self.current_heading = msg.vector.z

        if self.using_imu_control and self.target_heading is not None:
            error = self.normalize_angle(self.target_heading - self.current_heading)
            self.get_logger().debug(
                f"Heading: current={self.current_heading:.2f}, target={self.target_heading:.2f}, "
                f"error={error:.2f}"
            )

    def steer_to_target(self):
        if self.target_heading is None or not self.using_imu_control:
            return

        error = self.normalize_angle(self.target_heading - self.current_heading)

        current_time = time.time()
        derivative = 0.0

        if self.last_error_time is not None:
            dt = current_time - self.last_error_time
            if dt > 0:
                derivative = (error - self.prev_error) / dt

        self.prev_error = error
        self.last_error_time = current_time

        steering_adjustment = (self.Kp * error) + (self.Kd * derivative)

        steering_adjustment = max(
            -self.max_turn_degree, min(self.max_turn_degree, steering_adjustment)
        )

        self.s = self.mid - steering_adjustment

        pw = self.pwm(self.s)
        self.board.pwm_servo_set_position(0.1, [[1, pw]])

    def start_driving_forward(self):
        self.get_logger().info("Starting forward drive")
        self.state = "DRIVING_FORWARD"
        self.state_start_time = time.time()
        self.b = 1380
        self.s = self.mid
        self.using_imu_control = False
        self.update_controls()

    def start_turn(self):
        self.get_logger().info(f"Starting turn {self.completed_turns + 1}/4")
        self.state = "TURNING"

        initial_heading = self.current_heading

        self.target_heading = self.normalize_angle(initial_heading - math.pi / 2)

        self.get_logger().info(
            f"Turn {self.completed_turns + 1}: Initial heading: {initial_heading:.2f}, "
            f"Target heading: {self.target_heading:.2f}"
        )

        self.using_imu_control = True
        self.prev_error = 0.0
        self.last_error_time = None

        self.b = 1380

    def update_controls(self):
        pw = self.pwm(self.s)
        self.board.pwm_servo_set_position(0.1, [[2, self.b]])
        self.board.pwm_servo_set_position(0.1, [[1, pw]])

    def state_machine(self):
        if not self.running:
            return

        if self.state == "IDLE":
            self.start_driving_forward()

        elif self.state == "DRIVING_FORWARD":
            if time.time() - self.state_start_time >= self.straight_drive_duration:
                self.start_turn()

        elif self.state == "TURNING":
            self.steer_to_target()

            if self.target_heading is not None:
                error = abs(
                    self.normalize_angle(self.target_heading - self.current_heading)
                )
                if error < 0.1:
                    self.completed_turns += 1
                    self.get_logger().info(f"Turn {self.completed_turns} completed")

                    if self.completed_turns >= self.target_turns:
                        self.get_logger().info("Rectangular path completed!")
                        self.stop_robot()
                        self.state = "COMPLETE"
                    else:
                        self.start_driving_forward()

        elif self.state == "COMPLETE":
            self.b = 1500
            self.update_controls()

            if self.running:
                self.get_logger().info("Path complete. Press Ctrl+C to exit.")
                self.running = False


def main(args=None):
    rclpy.init(args=args)
    node = RectangularPathNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
