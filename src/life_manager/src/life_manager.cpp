#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/qos.hpp"

using namespace std::chrono_literals;

// Holds configuration and runtime data for each managed node
struct NodeData {
    double threshold;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
    rclcpp::TimerBase::SharedPtr timer; // watchdog
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> change_state_client;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> get_state_client;    
};

class LifeManager : public rclcpp::Node {
public:
    LifeManager() : rclcpp::Node("life_manager") {
        client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        declare_and_get_parameters();
        initialize_nodes();
        RCLCPP_INFO(this->get_logger(), "Life Manager started.");
    }

private:
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    std::unordered_map<std::string, NodeData> data_map_;
    std::vector<std::string> nodes_;
    int k_;

    // Declare and retrieve parameters from the parameter server
    void declare_and_get_parameters() {
        this->declare_parameter<int>("k", 3);
        this->declare_parameter<std::vector<std::string>>("nodes", {});
        k_ = this->get_parameter("k").as_int();
        nodes_ = this->get_parameter("nodes").as_string_array();
    }

    // Initialize subscriptions, timers, and service clients for each node
    void initialize_nodes() {
        for (const auto &node_name : nodes_) {
            double threshold = setup_node_parameters(node_name);
            auto sub = create_heartbeat_subscription(node_name);
            auto timer = create_heartbeat_timer(node_name, threshold);

            // Create service clients
            auto change_state_client = this->create_client<lifecycle_msgs::srv::ChangeState>(
                node_name + "/change_state",
                rmw_qos_profile_services_default,
                client_cb_group_
            );
            auto get_state_client = this->create_client<lifecycle_msgs::srv::GetState>(
                node_name + "/get_state",
                rmw_qos_profile_services_default,
                client_cb_group_
            );

            data_map_[node_name] = {threshold, sub, timer, change_state_client, get_state_client};

            RCLCPP_INFO(this->get_logger(), "Created change_state and get_state clients for: %s", node_name.c_str());
        }
    }

    // Setup node-specific parameters and compute threshold
    double setup_node_parameters(const std::string &node_name) {
        std::string period_param = "node_config." + node_name + ".period";
        int node_period_ms = this->declare_parameter<int>(period_param, 1000);
        double threshold = (node_period_ms / 1000.0) * k_;
        return threshold;
    }

    // Create a subscription for heartbeat messages
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr create_heartbeat_subscription(const std::string &node_name) {
        auto topic_name = "/heartbeat/" + node_name;
        auto sub = this->create_subscription<std_msgs::msg::String>(
            topic_name, rclcpp::QoS(10),
            [this, node_name](const std_msgs::msg::String::SharedPtr msg) {
                this->heartbeat_callback(msg, node_name);
            }
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", topic_name.c_str());
        return sub;
    }

    // Create a timer to detect missed heartbeats
    rclcpp::TimerBase::SharedPtr create_heartbeat_timer(const std::string &node_name, double threshold) {
        auto timer = this->create_wall_timer(
            std::chrono::duration<double>(threshold),
            [this, node_name]() {
                this->heartbeat_timeout(node_name);
            },
            client_cb_group_
        );
        return timer;
    }

    // Helper function to wait for async service results
    template<typename FutureT, typename WaitTimeT>
    std::future_status wait_for_result(
        FutureT & future,
        WaitTimeT time_to_wait)
    {
        auto end = std::chrono::steady_clock::now() + time_to_wait;
        std::chrono::milliseconds wait_period(100);
        std::future_status status = std::future_status::timeout;
        do {
            auto now = std::chrono::steady_clock::now();
            auto time_left = end - now;
            if (time_left <= std::chrono::seconds(0)) {break;}
            status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
        } while (rclcpp::ok() && status != std::future_status::ready);
        return status;
    }

    uint8_t get_node_state(const std::string &node_name) {
        auto it = data_map_.find(node_name);
        if (it == data_map_.end() || !it->second.get_state_client) {
            RCLCPP_ERROR(this->get_logger(), "No get_state client for node %s", node_name.c_str());
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }
        auto client = it->second.get_state_client;
        //print client name
        RCLCPP_INFO(this->get_logger(), "Client name: %s", client->get_service_name());
    
        if (!client->wait_for_service(3s)) {
            RCLCPP_ERROR(this->get_logger(), "Service %s not available.", client->get_service_name());
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }
    
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto future_result = client->async_send_request(request).future.share();        
        auto future_status = wait_for_result(future_result, 3s);

        if (future_status != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "\033[1;33mServer time out while changing state for node %s\033[0m", node_name.c_str());
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        // Get the result from the shared_future
        if (future_result.get()) {
            RCLCPP_INFO(this->get_logger(), "[%s] Current state: %u (%s)", node_name.c_str(),
                        future_result.get()->current_state.id,
                        future_result.get()->current_state.label.c_str());
            return future_result.get()->current_state.id;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get current state for node %s", node_name.c_str());
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }
    }

    bool change_state(const std::string &node_name, uint8_t transition) {
        auto it = data_map_.find(node_name);
        if (it == data_map_.end() || !it->second.change_state_client) {
            RCLCPP_ERROR(this->get_logger(), "No change_state client for node %s", node_name.c_str());
            return false;
        }
        auto client = it->second.change_state_client;
        
        if (!client->wait_for_service(3s)) {
            RCLCPP_ERROR(this->get_logger(), "Service %s not available.", client->get_service_name());
            return false;
        }
        
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;
        
        // Use shared_future with .share() to properly wait for the result
        auto future_result = client->async_send_request(request).future.share();
        
        // Wait for the result with the helper function
        auto future_status = wait_for_result(future_result, 3s);

        if (future_status != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "\033[1;33mServer time out while changing state for node %s\033[0m", node_name.c_str());
            return false;
        }

        // Get the result from the shared_future
        if (future_result.get()->success) {
            RCLCPP_INFO(this->get_logger(), "[%s] Successfully transitioned (%u).", 
                       node_name.c_str(), transition);
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "[%s] Failed to transition (%u).", 
                        node_name.c_str(), transition);
            return false;
        }
    }

    // Called when a heartbeat is missed
    void heartbeat_timeout(const std::string &node_name) {
        RCLCPP_WARN(this->get_logger(), "\033[1;31m[%s] Missed heartbeat threshold! Taking action.\033[0m", node_name.c_str());
    
        uint8_t state = get_node_state(node_name);
        if (state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN) {
            RCLCPP_INFO(this->get_logger(), "Current state for node %s: %u", node_name.c_str(), state);
            if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
                if (change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
                    RCLCPP_INFO(this->get_logger(), "\033[1;32m[%s] Node configured successfully.\033[0m", node_name.c_str());
                    // After configuring, try to activate
                    uint8_t new_state = get_node_state(node_name);
                    if (new_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                        if (change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
                            RCLCPP_INFO(this->get_logger(), "\033[1;32m[%s] Node activated successfully.\033[0m", node_name.c_str());
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "\033[1;31m[%s] Failed to activate node after configuring.\033[0m", node_name.c_str());
                        }
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "\033[1;31m[%s] Failed to configure node.\033[0m", node_name.c_str());
                }
            } else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                if (change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
                    RCLCPP_INFO(this->get_logger(), "\033[1;32m[%s] Node activated successfully.\033[0m", node_name.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "\033[1;31m[%s] Failed to activate node.\033[0m", node_name.c_str());
                }
            } else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                // If the node is active, try to deactivate and then reactivate
                if (change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
                    RCLCPP_WARN(this->get_logger(), "\033[1;33m[%s] Node deactivated due to missed heartbeat. Attempting to reactivate.\033[0m", node_name.c_str());
                    // After deactivation, try to activate again
                    uint8_t new_state = get_node_state(node_name);
                    if (new_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                        if (change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
                            RCLCPP_INFO(this->get_logger(), "\033[1;32m[%s] Node re-activated successfully after deactivation.\033[0m", node_name.c_str());
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "\033[1;31m[%s] Failed to re-activate node after deactivation.\033[0m", node_name.c_str());
                        }
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "\033[1;31m[%s] Failed to deactivate node.\033[0m", node_name.c_str());
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Could not determine current state for node %s", node_name.c_str());
        }
    }

    // Called when a heartbeat is received
    void heartbeat_callback(const std_msgs::msg::String::SharedPtr msg, const std::string &node_name) {
        auto it = data_map_.find(node_name);
        if (it != data_map_.end()) {
            it->second.timer->cancel();
            it->second.timer->reset();
            RCLCPP_INFO(this->get_logger(), "\033[1;32m[%s] Heartbeat received: '%s'\033[0m", node_name.c_str(), msg->data.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Received heartbeat from unknown node: %s", node_name.c_str());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto life_manager_node = std::make_shared<LifeManager>();
    executor.add_node(life_manager_node);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Spinning LifeManager node.");
    executor.spin();
    RCLCPP_INFO(rclcpp::get_logger("main"), "LifeManager node stopped.");
    rclcpp::shutdown();
    return 0;
}