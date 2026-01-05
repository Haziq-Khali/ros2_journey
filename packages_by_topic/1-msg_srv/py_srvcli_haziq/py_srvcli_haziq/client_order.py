import sys
import rclpy
from rclpy.node import Node

from tutorial_interfaces_haziq.srv import SetOrder   # import your srv


class OrderClientAsync(Node):

    def __init__(self):
        super().__init__('order_client_async')
        self.cli = self.create_client(SetOrder, 'set_order')

        # Wait until service is available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = SetOrder.Request()

    def send_request(self, name, set_item, quantity):
        self.req.name = name
        self.req.set = set_item
        self.req.quantity = quantity
        return self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    order_client = OrderClientAsync()

    # ---- Get values from terminal args ----
    # Example: ros2 run py_srvcli_haziq client Haziq "Nasi Lemak" 3
    if len(sys.argv) != 4:
        print("Usage: ros2 run py_srvcli_haziq client <name> <set> <quantity>")
        return

    name = sys.argv[1]
    set_item = sys.argv[2]
    quantity = int(sys.argv[3])

    future = order_client.send_request(name, set_item, quantity)
    rclpy.spin_until_future_complete(order_client, future)

    if future.result() is not None:
        response = future.result()
        order_client.get_logger().info(
            f"Order confirmed: {response.order}, Total items: {response.total_item}"
        )
    else:
        order_client.get_logger().error('Service call failed!')

    order_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
