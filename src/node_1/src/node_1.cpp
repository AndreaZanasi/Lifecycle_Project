#include "../include/node_1.hpp"
#include "../../signal_handler.hpp"

static sigjmp_buf g_jmpbuf;

bool safe_execute(const std::function<void()> &action)
{
    SignalHandler::set_jump_buffer(g_jmpbuf);
    if (sigsetjmp(g_jmpbuf, 1) == 0) {
        action();
        return true;
    }
    throw std::runtime_error("caught fatal signal");
}

void div_by_zero()    { volatile int x = 1 / 0; (void)x; }
void abort_now()      { std::abort(); }
void read_null()      { volatile int value = *static_cast<int*>(nullptr); (void)value; }
void write_null()     { *static_cast<int*>(nullptr) = 42; }
void overflow_array() { volatile int arr[5]; volatile int v = arr[1000000]; (void)v; }

Node1::Node1(const std::string &name, bool intra) 
    : BaseTalkerNode(name, intra), heartbeat_count_(0)
{
    declare_parameters();
}

void Node1::declare_parameters() {
    declare_parameter<std::string>("node_name", "node_1");
    declare_parameter<int>("period", 1000);
}

void Node1::load_parameters() {
    node_name_ = get_parameter("node_name").as_string();
    period_ = std::chrono::milliseconds(get_parameter("period").as_int());
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
Node1::on_configure(const rclcpp_lifecycle::State &)
{
    load_parameters();
    log_current_state();
    pub_ = create_publisher();
    timer_ = create_timer(std::bind(&Node1::publish_heartbeat, this));
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
Node1::on_activate(const rclcpp_lifecycle::State &)
{
    log_current_state();
    pub_->on_activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
Node1::on_deactivate(const rclcpp_lifecycle::State &)
{
    log_current_state();
    pub_->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
Node1::on_cleanup(const rclcpp_lifecycle::State &)
{
    log_current_state();
    timer_.reset();
    pub_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
Node1::on_shutdown(const rclcpp_lifecycle::State &)
{
    log_current_state();
    timer_.reset();
    pub_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void Node1::publish_heartbeat()
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

    if (heartbeat_count_ % 15 == 0) {
        const auto &fault = cases[(heartbeat_count_ / 5) % cases.size()];
        try {
            safe_execute(fault.action);
        } catch (const std::exception &) {
            RCLCPP_FATAL(get_logger(), fault.log_msg, heartbeat_count_);
        }
    }
}

int main(int argc, char **argv)
{
    SignalHandler::init();

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<Node1>("node_1");

    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}