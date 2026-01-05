import rclpy
from rclpy.node import Node
from tutorial_interfaces_haziq.srv import SetOrder


class OrderService(Node):

    def __init__(self):
        super().__init__('order_service')
        self.srv = self.create_service(SetOrder, 'set_order', self.set_order_callback)

    def set_order_callback(self, request, response):
        response.order = f"{request.set} for {request.name}"
        response.total_item = request.quantity
        self.get_logger().info(
            f"Received order: name={request.name}, set={request.set}, quantity={request.quantity}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = OrderService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
