#include <chrono>
#include <csignal>
#include <csetjmp>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/qos.hpp"
#include "../../base_talker_node.cpp"

using namespace std::chrono_literals;
static sigjmp_buf g_jmpbuf;

void fpe_handler(int, siginfo_t*, void*)
{
  siglongjmp(g_jmpbuf, 1);
}

static bool safe_divide_and_recover()
{
  if (sigsetjmp(g_jmpbuf, 1) == 0) {
    volatile int zero = 0;
    volatile int result = 1 / zero;
    (void)result;
    return true;
  } else {
    return false;
  }
}

// Node1: Inherit from BaseTalkerNode
class Node1 : public BaseTalkerNode
{
public:
  explicit Node1(const std::string & node_name, bool intra_process_comms = false)
  : BaseTalkerNode(node_name, intra_process_comms)
  {
    declare_parameters();
  }

  void declare_parameters() override {
    this->declare_parameter<std::string>("node_name", "node_1");
    this->declare_parameter<int>("period", 1000);  // ms
  }

  void load_parameters() override {
    node_name_ = this->get_parameter("node_name").as_string();
    period_ = std::chrono::milliseconds(this->get_parameter("period").as_int());
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override
  {
    load_parameters();
    log_current_state();

    pub_ = create_publisher();
    timer_ = create_timer(std::bind(&Node1::publish_heartbeat, this));

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override
  {
    log_current_state();
    pub_->on_activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    log_current_state();
    pub_->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override
  {
    log_current_state();
    timer_.reset();
    pub_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) override
  {
    log_current_state();
    timer_.reset();
    pub_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  void publish_heartbeat()
  {
    publish_message(pub_);

    heartbeat_count_++;
    if (heartbeat_count_ % 10 == 0) {
      if (!safe_divide_and_recover()) {
        RCLCPP_FATAL(this->get_logger(), "\033[1;31mCaught SIGFPE divide-by-zero! Deactivating node.\033[0m");
        this->deactivate();
      }
    }
  }

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
  int heartbeat_count_ = 0;
};

int main(int argc, char * argv[])
{
  struct sigaction sa{};
  sa.sa_sigaction = fpe_handler;
  sa.sa_flags     = SA_SIGINFO;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGFPE, &sa, nullptr);

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<Node1>("node_1");
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}