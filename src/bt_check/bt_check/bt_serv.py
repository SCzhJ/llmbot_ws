from cust_com.srv import XmlTree
from bt_check import valid_format
from bt_check import check_bt
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(XmlTree, 'XmlTree', self.xml_tree_callback)
        self.bt_checker = check_bt.BTChecker(valid_format.valid_format.valid_actions, 
                                             valid_format.valid_format.valid_control_nodes)

    def xml_tree_callback(self, request, response):
        response.result = self.bt_checker.check_behavior_tree(request.tree)
        response.feedback = self.bt_checker.feedback
        write_tree = self.bt_checker.generate_xml_tree()

        if response.result:
            self.get_logger().info('The behavior tree is valid. Writing to bt_gen.xml')
            head = " <root BTCPP_format=\"4\" >\n<BehaviorTree ID=\"MainTree\">\n"
            tail = "</BehaviorTree>\n</root>"

            # write_tree = request.tree.replace("  ", " ")
            # write_tree = write_tree.replace(">", ">\n")
            with open('/home/fyp/llmbot_ws/src/DefBT/Tree/bt_gen.xml', 'w') as f:
                f.write(head + write_tree + "\n" + tail)

        self.bt_checker.feedback = ""
        self.get_logger().info('Incoming request:\n %s ..' % request.tree)
        return response

def main():
    rclpy.init()
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()