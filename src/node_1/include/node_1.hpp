#ifndef NODE_1_HPP
#define NODE_1_HPP

#include <chrono>
#include <csignal>
#include <csetjmp>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/qos.hpp"
#include "../../base_talker_node.cpp"  // Include the base class definition

struct FaultCase {
    std::function<void()> action;
    const char * log_msg;
};

class Node1 : public BaseTalkerNode
{
public:
    explicit Node1(const std::string &name, bool intra = false);

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
    int heartbeat_count_;
    int signal_recovery_count_;
};

bool safe_execute(const std::function<void()> &action);
void div_by_zero();
void abort_now();
void read_null();
void write_null();
void overflow_array();

#endif