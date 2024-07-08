from cust_com.srv import XmlTree                            # CHANGE
import sys
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(XmlTree, 'XmlTree')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = XmlTree.Request()                                   # CHANGE

    def send_request(self, _str_tree):
        self.req.tree = _str_tree
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    # str_tree = "xml_tree"
    with open('/home/fyp/llmbot_ws/src/bt_check/bt_check/input.txt', 'r') as file:
        str_tree = file.read().strip()

    minimal_client.send_request(str_tree)

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of xml_tree: %s ' %                                # CHANGE
                    response.feedback)  # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()