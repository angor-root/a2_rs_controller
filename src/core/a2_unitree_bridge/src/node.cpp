#include <memory>
#include <rclcpp/executor.hpp>
#include <rclcpp/executor_options.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include "bridge.hpp"

namespace a2 {
namespace bridge {

class Node : public rclcpp::Node {
public:
  explicit Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node("a2_bridge_node", options), bridge_(nullptr) {}

  void initialize() {
#ifdef A2_MODE_SIM
    bridge_ = std::make_unique<A2SimBridge>(this);
#else
    bridge_ = std::make_unique<A2RobotBridge>(this);
#endif
  }

private:
  std::unique_ptr<A2BridgeBase> bridge_;
};

}  // namespace bridge
}  // namespace a2

int main(int argc, char** argv) {
  try {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);

#ifdef A2_MODE_SIM
    unitree::robot::ChannelFactory::Instance()->Init(1, "lo");
#else
    unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
#endif

    auto node = std::make_shared<a2::bridge::Node>(options);
    node->get_logger().set_level(rclcpp::Logger::Level::Debug);
    node->initialize();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
    return 1;
  }
}
