#include <chrono>
#include <csignal>
#include <csetjmp>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/qos.hpp"
#include "../../base_talker_node.cpp"
#include "../../signal_handler.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
static sigjmp_buf g_jmpbuf;

static bool safe_execute(const std::function<void()> &action)
{
    SignalHandler::set_jump_buffer(g_jmpbuf);
    if (sigsetjmp(g_jmpbuf, 1) == 0) {
        action();
        return true;
    }
    return false;
}

static void div_by_zero()    { volatile int x = 1 / 0; (void)x; }
static void abort_now()      { std::abort(); }
static void read_null()      { volatile int value = *static_cast<int*>(nullptr); (void)value; }
static void write_null()     { *static_cast<int*>(nullptr) = 42; }
static void overflow_array() { volatile int arr[5]; volatile int v = arr[1000000]; (void)v; }

struct FaultCase {
    std::function<void()> action;
    const char * log_msg;
};

class Node1 : public BaseTalkerNode
{
public:
    explicit Node1(const std::string &name, bool intra = false)
        : BaseTalkerNode(name, intra)
    {
        declare_parameters();
    }

    void declare_parameters() override {
        declare_parameter<std::string>("node_name", "node_1");
        declare_parameter<int>("period", 1000);
    }

    void load_parameters() override {
        node_name_ = get_parameter("node_name").as_string();
        period_ = std::chrono::milliseconds(get_parameter("period").as_int());
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        load_parameters();
        log_current_state();
        pub_ = create_publisher();
        timer_ = create_timer(std::bind(&Node1::publish_heartbeat, this));
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        log_current_state();
        pub_->on_activate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        log_current_state();
        pub_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        log_current_state();
        timer_.reset();
        pub_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
    {
        log_current_state();
        timer_.reset();
        pub_.reset();
        return CallbackReturn::SUCCESS;
    }

private:
    void publish_heartbeat()
    {
        publish_message(pub_);
        ++heartbeat_count_;

        static const std::vector<FaultCase> cases = {
            {div_by_zero,    "\033[1;31m[Heartbeat %d] Caught SIGFPE (Division by Zero)! Node recovered.\033[0m"},
            {abort_now,      "\033[1;33m[Heartbeat %d] Caught SIGABRT (Abnormal Termination)! Node recovered.\033[0m"},
            {read_null,      "\033[1;35m[Heartbeat %d] Caught SIGSEGV (Null Pointer Read)! Node recovered.\033[0m"},
            {write_null,     "\033[1;36m[Heartbeat %d] Caught SIGSEGV (Null Pointer Write)! Node recovered.\033[0m"},
            {overflow_array, "\033[1;32m[Heartbeat %d] Caught SIGSEGV (Array Overflow)! Node recovered.\033[0m"}
        };

        if (heartbeat_count_ % 5 == 0) {
            const auto &fault = cases[(heartbeat_count_ / 5) % cases.size()];
            if (!safe_execute(fault.action)) {
                RCLCPP_FATAL(get_logger(), fault.log_msg, heartbeat_count_);
                ++signal_recovery_count_;
            }
        }

        if (heartbeat_count_ % 10 == 0) {
            RCLCPP_INFO(get_logger(),
                "\033[1;37m[Status] Heartbeat: %d, Signal recoveries: %d\033[0m",
                heartbeat_count_, signal_recovery_count_);
        }

        if (signal_recovery_count_ >= 10) {
            RCLCPP_WARN(get_logger(),
                "\033[1;31mToo many signal recoveries (%d)! Deactivating node for safety.\033[0m",
                signal_recovery_count_);
            deactivate();
        }
    }

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
    std::shared_ptr<rclcpp::TimerBase> timer_;
    int heartbeat_count_ = 0;
    int signal_recovery_count_ = 0;
};

int main(int argc, char **argv)
{
    SignalHandler::init();
    RCLCPP_INFO(rclcpp::get_logger("main"),
        "\033[1;34mSignal Handler Demo Node Starting...\033[0m");
    RCLCPP_INFO(rclcpp::get_logger("main"),
        "\033[1;34mThis node will simulate SIGFPE, SIGABRT, and SIGSEGV errors\033[0m");

    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<Node1>("node_1");
    executor.add_node(node->get_node_base_interface());

    try {
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in executor: %s", e.what());
    }

    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("main"),
        "\033[1;34mSignal Handler Demo Node Shutting Down...\033[0m");
    return 0;
}
