#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <behaviortree_ros2/bt_action_node.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <chrono>
#include "cust_com/srv/run_tree.hpp"
#include <memory>

using namespace BT;

using N2P = nav2_msgs::action::NavigateToPose;
using GoalHandleN2P = rclcpp_action::ClientGoalHandle<N2P>;

using std::placeholders::_1; using std::placeholders::_2;

class MoveTo: public RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
    MoveTo(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);
    static PortsList providedPorts();
    bool setGoal(Goal& goal) override;
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);
    NodeStatus onResultReceived(const WrappedResult& wr) override;
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;
private:
    // Shared pointer to the ROS 2 node for communication
    std::shared_ptr<rclcpp::Node> client;
};

MoveTo::MoveTo(const std::string& name,
    const NodeConfig& conf,
    const RosNodeParams& params)
    : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
    client = params.nh.lock();
}

PortsList MoveTo::providedPorts()
{
    return {
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y")
    };
}

bool MoveTo::setGoal(RosActionNode::Goal &goal)
{

    double x, y;
    if (!getInput("x", x) || !getInput("y", y)) {
      return false;
    }
    goal.pose.header.stamp = client->now();
    goal.pose.header.frame_id = "map";
  
    // Set the position
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.position.z = 0.0;
  
    goal.pose.pose.orientation.x = 0;
    goal.pose.pose.orientation.y = 0;
    goal.pose.pose.orientation.z = 0;
    goal.pose.pose.orientation.w = 1;
  
    return true;
}

NodeStatus MoveTo::onResultReceived(const RosActionNode::WrappedResult &wr)
{   
    return NodeStatus::SUCCESS;
}

NodeStatus MoveTo::onFailure(ActionNodeErrorCode error)
{   
    return NodeStatus::FAILURE;
}

NodeStatus MoveTo::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
    return NodeStatus::RUNNING;
}



void run_tree(const std::shared_ptr<cust_com::srv::RunTree::Request> request,
          std::shared_ptr<cust_com::srv::RunTree::Response>       response,
          const std::shared_ptr<rclcpp::Node> node)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request\n");
    BehaviorTreeFactory factory;
    RosNodeParams params;
    params.nh = node;
    params.default_port_value = "/navigate_to_pose";
    factory.registerNodeType<MoveTo>("MoveTo", params);

    auto tree = factory.createTreeFromFile("/home/fyp/llmbot_ws/src/DefBT/Tree/bt_gen.xml");
    tree.tickWhileRunning();

    response->terminated = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response");

}


int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_client_node");

  rclcpp::Service<cust_com::srv::RunTree>::SharedPtr service =
    node->create_service<cust_com::srv::RunTree>("run_tree",  std::bind(run_tree, _1, _2, node));   // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready run tree");                     // CHANGE

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}