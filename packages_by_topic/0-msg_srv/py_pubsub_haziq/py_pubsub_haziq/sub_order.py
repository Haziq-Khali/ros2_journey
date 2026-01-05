import rclpy
from rclpy.node import Node
from tutorial_interfaces_haziq.msg import Order


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Order,
            'food_order',                   #Must match dgn pub
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Order):
        self.get_logger().info(
            f"Order terima: order_id [{msg.order_id}] order={msg.order}, quantity={msg.quantity}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
