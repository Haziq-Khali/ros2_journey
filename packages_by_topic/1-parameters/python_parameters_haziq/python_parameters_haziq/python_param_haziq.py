import rclpy
import rclpy.node

class HaziqParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('haziq_param_node')

        self.declare_parameter('my_parameter', 'world')
        self.declare_parameter('my_quantity', 9)
        self.declare_parameter('my_item', 'Nvidia Graphic')

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
        my_item = self.get_parameter('my_item').get_parameter_value().string_value
        my_quantity = self.get_parameter('my_quantity').get_parameter_value().integer_value

        self.get_logger().info(f'Hello {my_param}! We only sell {my_quantity} {my_item}')

        my_new_param = rclpy.parameter.Parameter(
            'my_parameter', rclpy.Parameter.Type.STRING, 'world',
            #'my_item', rclpy.Parameter.Type.STRING, 'Nvidia Graphic',
            #'my_quantity', rclpy.Parameter.Type.INTEGER, 9,
        )
        #all_new_parameters = [my_new_param]
        #self.set_parameters(all_new_parameters)

def main():
    rclpy.init()
    node = HaziqParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()