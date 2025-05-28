#include "../include/life_manager.hpp"

LifeManager::LifeManager() : rclcpp::Node("life_manager") {
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    declare_and_get_parameters();
    initialize_nodes();
    RCLCPP_INFO(this->get_logger(), "Life Manager started.");
}

void LifeManager::declare_and_get_parameters() {
    this->declare_parameter<int>("k", 3);
    this->declare_parameter<std::vector<std::string>>("nodes", {}, rcl_interfaces::msg::ParameterDescriptor{});
    k_ = this->get_parameter("k").as_int();
    nodes_ = this->get_parameter("nodes").as_string_array();
}

void LifeManager::initialize_nodes() {
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

double LifeManager::setup_node_parameters(const std::string &node_name) {
    std::string period_param = "node_config." + node_name + ".period";
    int node_period_ms = this->declare_parameter<int>(period_param, 1000);
    return (node_period_ms / 1000.0) * k_;
}

template<typename ServiceT>
std::shared_ptr<rclcpp::Client<ServiceT>> LifeManager::create_service_client(const std::string &node_name, const std::string &suffix) {
    return this->create_client<ServiceT>(
        node_name + suffix,
        rmw_qos_profile_services_default,
        client_cb_group_
    );
}

rclcpp::Subscription<std_msgs::msg::String>::SharedPtr LifeManager::create_heartbeat_subscription(const std::string &node_name) {
    auto topic_name = "/heartbeat/" + node_name;
    return this->create_subscription<std_msgs::msg::String>(
        topic_name, rclcpp::QoS(10),
        [this, node_name](const std_msgs::msg::String::SharedPtr msg) {
            this->on_heartbeat(msg, node_name);
        }
    );
}

rclcpp::TimerBase::SharedPtr LifeManager::create_heartbeat_timer(const std::string &node_name, double threshold) {
    return this->create_wall_timer(
        std::chrono::duration<double>(threshold),
        [this, node_name]() { this->on_heartbeat_timeout(node_name); },
        client_cb_group_
    );
}

void LifeManager::on_heartbeat(const std_msgs::msg::String::SharedPtr msg, const std::string &node_name) {
    auto it = data_map_.find(node_name);
    if (it != data_map_.end()) {
        it->second.watchdog->reset();
        RCLCPP_INFO(this->get_logger(), "\033[1;32m[%s] Heartbeat received: '%s'\033[0m", node_name.c_str(), msg->data.c_str());
    } else {
        log_unknown_node_warning(node_name);
    }
}

void LifeManager::on_heartbeat_timeout(const std::string &node_name) {
    RCLCPP_WARN(this->get_logger(), "\033[1;31m[%s] Missed heartbeat threshold! Taking action.\033[0m", node_name.c_str());
    recovery(node_name);
}

uint8_t LifeManager::get_node_state(const std::string &node_name) {
    auto client = get_client<lifecycle_msgs::srv::GetState>(node_name);
    if (!client) return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;

    if (!wait_for_service(client, std::chrono::seconds(3))) return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future_result = client->async_send_request(request).future.share();
    if (!wait_for_future(future_result, std::chrono::seconds(3))) return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;

    auto response = future_result.get();
    if (response) {
        RCLCPP_INFO(this->get_logger(), "[%s] Current state: \033[1;34m%u\033[0m (%s)", node_name.c_str(),
                    response->current_state.id, response->current_state.label.c_str());
        return response->current_state.id;
    }
    log_state_error(node_name);
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
}

bool LifeManager::change_state(const std::string &node_name, uint8_t transition) {
    auto client = get_client<lifecycle_msgs::srv::ChangeState>(node_name);
    if (!client) return false;

    if (!wait_for_service(client, std::chrono::seconds(3))) return false;

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    auto future_result = client->async_send_request(request).future.share();
    if (!wait_for_future(future_result, std::chrono::seconds(3))) return false;

    if (future_result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "[%s] Successfully transitioned (%u).", node_name.c_str(), transition);
        return true;
    }
    RCLCPP_ERROR(this->get_logger(), "[%s] Failed to transition (%u).", node_name.c_str(), transition);
    return false;
}

template<typename ServiceT>
std::shared_ptr<rclcpp::Client<ServiceT>> LifeManager::get_client(const std::string &node_name) {
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
bool LifeManager::wait_for_service(const std::shared_ptr<ClientT> &client, std::chrono::seconds timeout) {
    if (!client->wait_for_service(timeout)) {
        RCLCPP_ERROR(this->get_logger(), "Service %s not available.", client->get_service_name());
        return false;
    }
    return true;
}

template<typename FutureT>
bool LifeManager::wait_for_future(FutureT &future, std::chrono::seconds timeout) {
    auto status = wait_for_result(future, timeout);
    if (status != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "\033[1;33mServer time out while waiting for response\033[0m");
        return false;
    }
    return true;
}

template<typename FutureT, typename WaitTimeT>
std::future_status LifeManager::wait_for_result(FutureT &future, WaitTimeT time_to_wait) {
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

void LifeManager::recovery(const std::string &node_name) {
    uint8_t state = get_node_state(node_name);
    if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        bring_node_to_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, "deactivated");
        state = get_node_state(node_name);
    }
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

void LifeManager::bring_node_to_state(const std::string &node_name, uint8_t transition, const std::string &action) {
    if (change_state(node_name, transition)) {
        RCLCPP_INFO(this->get_logger(), "\033[1;32m[%s] Node %s successfully.\033[0m", node_name.c_str(), action.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "\033[1;31m[%s] Failed to %s node.\033[0m", node_name.c_str(), action.c_str());
    }
}

void LifeManager::bring_node_shutdown(const std::string &node_name) {
    uint8_t state = get_node_state(node_name);
    if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        bring_node_to_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, "deactivated");
        state = get_node_state(node_name);
    }
    if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        bring_node_to_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, "cleaned up");
        state = get_node_state(node_name);
    }
    if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        bring_node_to_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN, "shutdown");
    } else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN) {
        RCLCPP_WARN(this->get_logger(), "\033[1;33m[%s] Node is in UNKNOWN state, cannot shutdown.\033[0m", node_name.c_str());
    }
}

void LifeManager::log_unknown_node_warning(const std::string &node_name) {
    RCLCPP_WARN(this->get_logger(), "\033[1;33mNode %s not found in data map.\033[0m", node_name.c_str());
}

void LifeManager::log_state_error(const std::string &node_name) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get current state for node %s", node_name.c_str());
}

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