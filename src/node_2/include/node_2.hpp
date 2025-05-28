#ifndef NODE_2_HPP
#define NODE_2_HPP

#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/qos.hpp"
#include "../../base_talker_node.cpp"

class Node2 : public BaseTalkerNode
{
public:
    explicit Node2(const std::string &name, bool intra = false);

    void declare_parameters() override;
    void load_parameters() override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_configure(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_activate(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_deactivate(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_cleanup(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_shutdown(const rclcpp_lifecycle::State &) override;

private:
    void publish_heartbeat();

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
    std::shared_ptr<rclcpp::TimerBase> timer_;
};

#endif