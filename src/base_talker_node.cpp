#pragma once

#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class BaseTalkerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit BaseTalkerNode(const std::string & node_name, bool intra_process_comms = false)
    : rclcpp_lifecycle::LifecycleNode(
        node_name,
        rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {}

    virtual void declare_parameters() = 0;
    virtual void load_parameters() = 0;

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &) override = 0;

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &) override = 0;

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &) override = 0;

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &) override = 0;

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &) override = 0;

    void log_current_state() {
    auto state = this->get_current_state();
    RCLCPP_INFO(
        this->get_logger(),
        "Current lifecycle state: \033[1;36m[%s] (%d)\033[0m",
        state.label().c_str(),
        state.id());
    }

    void publish_message(std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_)
    {
        if (!pub_ || !pub_->is_activated()) {
            return;
        }
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = node_name_ + " heartbeat";
        RCLCPP_INFO(this->get_logger(), "Publishing: '\033[1;32m%s\033[0m'", msg->data.c_str());
        pub_->publish(std::move(msg));
    }

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> create_publisher() {
        return rclcpp_lifecycle::LifecycleNode::create_publisher<std_msgs::msg::String>("/heartbeat/" + node_name_, qos_profile);
    }

    std::shared_ptr<rclcpp::TimerBase> create_timer(std::function<void()> callback) {
        return this->create_wall_timer(period_, callback);
    }

    virtual ~BaseTalkerNode() = default;

protected:
  std::string node_name_;
  std::chrono::milliseconds period_;
  rclcpp::QoS qos_profile{rclcpp::KeepLast(10)};
};