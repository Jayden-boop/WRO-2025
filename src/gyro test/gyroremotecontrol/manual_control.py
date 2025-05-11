import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading


class ManualControl(Node):
    def __init__(self):
        super().__init__("manual_control")
        self.publisher = self.create_publisher(String, "/cmd_drive", 10)
        self.get_logger().info("Use keys: [W/A/S/D/X/Space], Space to stop and exit.")

        # Create a thread for input to avoid blocking ROS spin
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.running = True
        self.input_thread.start()

    def input_loop(self):
        while self.running:
            key = input("Give command: ")
            msg = String()

            if key in ["w", "a", "s", "d", "x"]:
                msg.data = key
                self.publisher.publish(msg)

                # Print the command description like in remotecontrol.py
                if key == "a":
                    print("right")
                elif key == "d":
                    print("left")
                elif key == "w":
                    print("forward")
                elif key == "s":
                    print("backward")
                elif key == "x":
                    print("straight")

            elif key == " ":
                # Stop command
                msg.data = "stop"
                self.publisher.publish(msg)
                print("stop")
                self.running = False
                # Request node shutdown
                rclpy.shutdown()
                break


def main(args=None):
    rclpy.init(args=args)
    node = ManualControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
