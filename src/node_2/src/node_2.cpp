#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/qos.hpp"
#include "../../base_talker_node.cpp"

using namespace std::chrono_literals;

// Node2: Inherit from BaseTalkerNode
class Node2 : public BaseTalkerNode
{
public:
  explicit Node2(const std::string & node_name, bool intra_process_comms = false)
  : BaseTalkerNode(node_name, intra_process_comms)
  {
    declare_parameters();
  }

  void declare_parameters() override {
    this->declare_parameter<std::string>("node_name", "node_2");
    this->declare_parameter<int>("period", 1000); // ms
  }

  void load_parameters() override {
    node_name_ = this->get_parameter("node_name").as_string();
    int period_ms = this->get_parameter("period").as_int();
    period_ = std::chrono::milliseconds(period_ms);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override
  {
    load_parameters();
    log_current_state();

    pub_ = create_publisher();
    timer_ = create_timer(std::bind(&Node2::publish_heartbeat, this));    
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override
  {
    log_current_state();
    pub_->on_activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    log_current_state();
    pub_->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override
  {
    log_current_state();
    timer_.reset();
    pub_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) override
  {
    log_current_state();
    timer_.reset();
    pub_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  void publish_heartbeat()
  {
    publish_message(pub_);
  }

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<Node2>("node_2");
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}