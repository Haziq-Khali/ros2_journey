import rclpy
from rclpy.node import Node

from tutorial_interfaces_haziq.msg import Order     #Change 


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Order, 'food_order', 10)        #Change (publish name, mesti match dgn sub)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 1

    def timer_callback(self):
        msg = Order()                                           #filename.msg
        msg.order_id = self.i
        msg.order = 'ayam goreng'                                #fieldname in msg file
        msg.quantity = 2
        self.publisher_.publish(msg)
        self.get_logger().info('Hi, saya nak order %s x%s [%d]' % (msg.order, msg.quantity, msg.order_id))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()