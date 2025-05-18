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
    rclcpp::TimerBase::SharedPtr watchdog;
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

    // Parameter handling
    void declare_and_get_parameters() {
        this->declare_parameter<int>("k", 3);
        this->declare_parameter<std::vector<std::string>>("nodes", {}, rcl_interfaces::msg::ParameterDescriptor{});
        k_ = this->get_parameter("k").as_int();
        nodes_ = this->get_parameter("nodes").as_string_array();
    }

    // Node initialization
    void initialize_nodes() {
        for (const auto &node_name : nodes_) {
            double threshold = setup_node_parameters(node_name);
            data_map_[node_name] = {
                threshold,
                create_heartbeat_subscription(node_name),
                create_heartbeat_timer(node_name, threshold),
                create_service_client<lifecycle_msgs::srv::ChangeState>(node_name, "/change_state"),
                create_service_client<lifecycle_msgs::srv::GetState>(node_name, "/get_state")
            };
            RCLCPP_INFO(this->get_logger(), "Initialized node: %s", node_name.c_str());
        }
    }

    double setup_node_parameters(const std::string &node_name) {
        std::string period_param = "node_config." + node_name + ".period";
        int node_period_ms = this->declare_parameter<int>(period_param, 1000);
        return (node_period_ms / 1000.0) * k_;
    }

    // Generic service client creator
    template<typename ServiceT>
    std::shared_ptr<rclcpp::Client<ServiceT>> create_service_client(const std::string &node_name, const std::string &suffix) {
        return this->create_client<ServiceT>(
            node_name + suffix,
            rmw_qos_profile_services_default,
            client_cb_group_
        );
    }

    // Heartbeat subscription and timer
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr create_heartbeat_subscription(const std::string &node_name) {
        auto topic_name = "/heartbeat/" + node_name;
        return this->create_subscription<std_msgs::msg::String>(
            topic_name, rclcpp::QoS(10),
            [this, node_name](const std_msgs::msg::String::SharedPtr msg) {
                this->on_heartbeat(msg, node_name);
            }
        );
    }

    rclcpp::TimerBase::SharedPtr create_heartbeat_timer(const std::string &node_name, double threshold) {
        return this->create_wall_timer(
            std::chrono::duration<double>(threshold),
            [this, node_name]() { this->on_heartbeat_timeout(node_name); },
            client_cb_group_
        );
    }

    // Heartbeat handling
    void on_heartbeat(const std_msgs::msg::String::SharedPtr msg, const std::string &node_name) {
        auto it = data_map_.find(node_name);
        if (it != data_map_.end()) {
            it->second.watchdog->cancel();
            it->second.watchdog->reset();
            RCLCPP_INFO(this->get_logger(), "\033[1;32m[%s] Heartbeat received: '%s'\033[0m", node_name.c_str(), msg->data.c_str());
        } else {
            log_unknown_node_warning(node_name);
        }
    }

    void on_heartbeat_timeout(const std::string &node_name) {
        RCLCPP_WARN(this->get_logger(), "\033[1;31m[%s] Missed heartbeat threshold! Taking action.\033[0m", node_name.c_str());
        recovery(node_name);
    }

    // Lifecycle state helpers
    uint8_t get_node_state(const std::string &node_name) {
        auto client = get_client<lifecycle_msgs::srv::GetState>(node_name);
        if (!client) return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;

        if (!wait_for_service(client, 3s)) return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;

        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto future_result = client->async_send_request(request).future.share();
        if (!wait_for_future(future_result, 3s)) return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;

        auto response = future_result.get();
        if (response) {
            RCLCPP_INFO(this->get_logger(), "[%s] Current state: \033[1;34m%u\033[0m (%s)", node_name.c_str(),
                        response->current_state.id, response->current_state.label.c_str());
            return response->current_state.id;
        }
        log_state_error(node_name);
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    bool change_state(const std::string &node_name, uint8_t transition) {
        auto client = get_client<lifecycle_msgs::srv::ChangeState>(node_name);
        if (!client) return false;

        if (!wait_for_service(client, 3s)) return false;

        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;
        auto future_result = client->async_send_request(request).future.share();
        if (!wait_for_future(future_result, 3s)) return false;

        if (future_result.get()->success) {
            RCLCPP_INFO(this->get_logger(), "[%s] Successfully transitioned (%u).", node_name.c_str(), transition);
            return true;
        }
        RCLCPP_ERROR(this->get_logger(), "[%s] Failed to transition (%u).", node_name.c_str(), transition);
        return false;
    }

    // Template helpers for clients and waiting
    template<typename ServiceT>
    std::shared_ptr<rclcpp::Client<ServiceT>> get_client(const std::string &node_name) {
        auto it = data_map_.find(node_name);
        if (it == data_map_.end()) {
            log_unknown_node_warning(node_name);
            return nullptr;
        }
        if constexpr (std::is_same<ServiceT, lifecycle_msgs::srv::ChangeState>::value) {
            return it->second.change_state_client;
        } else if constexpr (std::is_same<ServiceT, lifecycle_msgs::srv::GetState>::value) {
            return it->second.get_state_client;
        }
        return nullptr;
    }

    template<typename ClientT>
    bool wait_for_service(const std::shared_ptr<ClientT> &client, std::chrono::seconds timeout) {
        if (!client->wait_for_service(timeout)) {
            RCLCPP_ERROR(this->get_logger(), "Service %s not available.", client->get_service_name());
            return false;
        }
        return true;
    }

    template<typename FutureT>
    bool wait_for_future(FutureT &future, std::chrono::seconds timeout) {
        auto status = wait_for_result(future, timeout);
        if (status != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "\033[1;33mServer time out while waiting for response\033[0m");
            return false;
        }
        return true;
    }

    template<typename FutureT, typename WaitTimeT>
    std::future_status wait_for_result(FutureT &future, WaitTimeT time_to_wait) {
        auto end = std::chrono::steady_clock::now() + time_to_wait;
        std::chrono::milliseconds wait_period(100);
        std::future_status status = std::future_status::timeout;
        do {
            auto now = std::chrono::steady_clock::now();
            auto time_left = end - now;
            if (time_left <= std::chrono::seconds(0)) break;
            status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
        } while (rclcpp::ok() && status != std::future_status::ready);
        return status;
    }

    // Lifecycle transitions
    void recovery(const std::string &node_name) {
        uint8_t state = get_node_state(node_name);
        if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
            bring_node_to_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, "configured");
            state = get_node_state(node_name);
        }
        if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            bring_node_to_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "activated");
            state = get_node_state(node_name);
        }
        if (state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            RCLCPP_WARN(this->get_logger(), "Could not determine current state for node %s", node_name.c_str());
        }
    }

    void bring_node_to_state(const std::string &node_name, uint8_t transition, const std::string &action) {
        if (change_state(node_name, transition)) {
            RCLCPP_INFO(this->get_logger(), "\033[1;32m[%s] Node %s successfully.\033[0m", node_name.c_str(), action.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "\033[1;31m[%s] Failed to %s node.\033[0m", node_name.c_str(), action.c_str());
        }
    }

    // Additional transitions
    void bring_node_deactivate(const std::string &node_name) {
        uint8_t state = get_node_state(node_name);
        if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            bring_node_to_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, "deactivated");
        } else {
            RCLCPP_WARN(this->get_logger(), "\033[1;33m[%s] Node is not in ACTIVE state.\033[0m", node_name.c_str());
        }
    }

    void bring_node_cleanup(const std::string &node_name) {
        uint8_t state = get_node_state(node_name);
        if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            bring_node_to_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, "cleaned up");
        } else {
            RCLCPP_WARN(this->get_logger(), "\033[1;33m[%s] Node is not in ACTIVE state.\033[0m", node_name.c_str());
        }
    }

    void bring_node_shutdown(const std::string &node_name) {
        uint8_t state = get_node_state(node_name);
        if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            bring_node_deactivate(node_name);
            state = get_node_state(node_name);
        }
        if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            bring_node_cleanup(node_name);
            state = get_node_state(node_name);
        }
        if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
            bring_node_to_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN, "shutdown");
        } else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN) {
            RCLCPP_WARN(this->get_logger(), "\033[1;33m[%s] Node is in UNKNOWN state, cannot shutdown.\033[0m", node_name.c_str());
        }
    }

    // Logging helpers
    void log_unknown_node_warning(const std::string &node_name) {
        RCLCPP_WARN(this->get_logger(), "\033[1;33mNode %s not found in data map.\033[0m", node_name.c_str());
    }
    void log_state_error(const std::string &node_name) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current state for node %s", node_name.c_str());
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