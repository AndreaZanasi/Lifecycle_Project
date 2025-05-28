#include "../include/node_2.hpp"

Node2::Node2(const std::string &name, bool intra) 
    : BaseTalkerNode(name, intra)
{
    declare_parameters();
}

void Node2::declare_parameters() {
    declare_parameter<std::string>("node_name", "node_2");
    declare_parameter<int>("period", 1000);
}

void Node2::load_parameters() {
    node_name_ = get_parameter("node_name").as_string();
    period_ = std::chrono::milliseconds(get_parameter("period").as_int());
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Node2::on_configure(const rclcpp_lifecycle::State &)
{
    load_parameters();
    log_current_state();
    pub_ = create_publisher();
    timer_ = create_timer(std::bind(&Node2::publish_heartbeat, this));
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Node2::on_activate(const rclcpp_lifecycle::State &)
{
    log_current_state();
    pub_->on_activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Node2::on_deactivate(const rclcpp_lifecycle::State &)
{
    log_current_state();
    pub_->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Node2::on_cleanup(const rclcpp_lifecycle::State &)
{
    log_current_state();
    timer_.reset();
    pub_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Node2::on_shutdown(const rclcpp_lifecycle::State &)
{
    log_current_state();
    timer_.reset();
    pub_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void Node2::publish_heartbeat()
{
    publish_message(pub_);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<Node2>("node_2");
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}