#ifndef LIFE_MANAGER_HPP
#define LIFE_MANAGER_HPP

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <future>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/qos.hpp"

struct NodeData {
    double threshold;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
    rclcpp::TimerBase::SharedPtr watchdog;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> change_state_client;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> get_state_client;    
};

class LifeManager : public rclcpp::Node {
public:
    LifeManager();

private:
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    std::unordered_map<std::string, NodeData> data_map_;
    std::vector<std::string> nodes_;
    int k_;

    void declare_and_get_parameters();
    void initialize_nodes();
    double setup_node_parameters(const std::string &node_name);

    template<typename ServiceT>
    std::shared_ptr<rclcpp::Client<ServiceT>> create_service_client(const std::string &node_name, const std::string &suffix);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr create_heartbeat_subscription(const std::string &node_name);
    rclcpp::TimerBase::SharedPtr create_heartbeat_timer(const std::string &node_name, double threshold);

    void on_heartbeat(const std_msgs::msg::String::SharedPtr msg, const std::string &node_name);
    void on_heartbeat_timeout(const std::string &node_name);

    uint8_t get_node_state(const std::string &node_name);
    bool change_state(const std::string &node_name, uint8_t transition);

    template<typename ServiceT>
    std::shared_ptr<rclcpp::Client<ServiceT>> get_client(const std::string &node_name);

    template<typename ClientT>
    bool wait_for_service(const std::shared_ptr<ClientT> &client, std::chrono::seconds timeout);

    template<typename FutureT>
    bool wait_for_future(FutureT &future, std::chrono::seconds timeout);

    template<typename FutureT, typename WaitTimeT>
    std::future_status wait_for_result(FutureT &future, WaitTimeT time_to_wait);

    void recovery(const std::string &node_name);
    void bring_node_to_state(const std::string &node_name, uint8_t transition, const std::string &action);
    void bring_node_deactivate(const std::string &node_name);
    void bring_node_cleanup(const std::string &node_name);
    void bring_node_shutdown(const std::string &node_name);

    void log_unknown_node_warning(const std::string &node_name);
    void log_state_error(const std::string &node_name);
};

#endif