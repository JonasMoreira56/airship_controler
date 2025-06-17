import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DirigibleController(Node):
    def __init__(self):
        super().__init__('dirigible_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self.move_dirigible, 10)
        self.get_logger().info('Dirigible Controller Node Started')

    def move_dirigible(self, msg):
        self.get_logger().info(f'Moving Dirigible: Linear X: {msg.linear.x}, Linear Y: {msg.linear.y}, Angular Z: {msg.angular.z}')
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DirigibleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()