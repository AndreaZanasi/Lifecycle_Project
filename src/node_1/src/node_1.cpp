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

// Node1: Lifecycle node that periodically publishes heartbeat messages
class Node1 : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit Node1(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(
      node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {
    declare_parameters();
  }

  // Configure lifecycle: setup publisher and timer
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override
  {
    rclcpp::QoS qos_profile{rclcpp::KeepLast(10)};
    load_parameters();

    RCLCPP_INFO(get_logger(),
                "Configuring node: %s, heartbeat period: %ld ms",
                node_name_.c_str(), period_.count());

    pub_ = this->create_publisher<std_msgs::msg::String>(
      "/heartbeat/" + node_name_, qos_profile);
    timer_ = this->create_wall_timer(
      period_, std::bind(&Node1::publish_heartbeat, this));
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Activate lifecycle: activate publisher
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Activating node: %s", node_name_.c_str());
    pub_->on_activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Deactivate lifecycle: deactivate publisher
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Deactivating node: %s", node_name_.c_str());
    pub_->on_deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Cleanup lifecycle: release resources
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Cleaning up node: %s", node_name_.c_str());
    timer_.reset();
    pub_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Shutdown lifecycle: release resources
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(),
                "Shutting down node: %s from state %s",
                node_name_.c_str(), state.label().c_str());
    timer_.reset();
    pub_.reset();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  void declare_parameters() {
    this->declare_parameter<std::string>("node_name", "node_1");
    this->declare_parameter<int>("period", 1000);  // ms
  }

  void load_parameters() {
    node_name_ = this->get_parameter("node_name").as_string();
    int period_ms = this->get_parameter("period").as_int();
    period_ = std::chrono::milliseconds(period_ms);
  }

  void publish_heartbeat()
  {
    if (!pub_ || !pub_->is_activated()) {
      return;
    }

    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = node_name_ + " heartbeat";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());
    pub_->publish(std::move(msg));

    heartbeat_count_++;
    if (heartbeat_count_ % 10 == 0) {
      RCLCPP_WARN(this->get_logger(), "Attempting real divide-by-zero…");
      if (!safe_divide_and_recover()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Caught SIGFPE divide-by-zero! Deactivating node.");
        this->deactivate();
      }
    }
  }

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
  std::string node_name_;
  std::chrono::milliseconds period_;
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
