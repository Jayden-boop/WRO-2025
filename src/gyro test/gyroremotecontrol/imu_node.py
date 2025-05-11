import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.subscription = self.create_subscription(
            Vector3Stamped,
            '/imu/rpy/filtered',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Filtered RPY - z: {msg.vector.z:.4f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
