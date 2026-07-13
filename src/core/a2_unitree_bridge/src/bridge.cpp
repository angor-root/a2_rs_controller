#include "bridge.hpp"

namespace a2 {
namespace bridge {

#ifndef A2_MODE_SIM
A2RobotBridge::A2RobotBridge(rclcpp::Node* node) {
  RCLCPP_DEBUG(node->get_logger(), "Initialize Robot Bridge");
  command_egress_.init(node);

  low_state_topic_.init(node);
  sport_state_topic_.init(node);
}
#endif  // !A2_MODE_SIM

#ifdef A2_MODE_SIM
A2SimBridge::A2SimBridge(rclcpp::Node* node) {
  RCLCPP_DEBUG(node->get_logger(), "Initialize Sim Bridge");
  rclcpp::SubscriptionOptions sub_reentrant_options;
  sub_reentrant_options.callback_group =
    node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  bool publish_odom = true;
  node->get_parameter_or("publish_odom", publish_odom, true);

  low_cmd_pub_topic_.init(node, sub_reentrant_options);

  low_state_topic_.init(node, cached_quat_, cached_ang_vel_, &cached_stamp_, &cache_mutex_);
  sport_state_topic_.init(node, cached_quat_, cached_ang_vel_, &cached_stamp_, &cache_mutex_);
  camera_topic_.init(node);
  front_lidar_topic_.init(node, /*with_registered_scan=*/publish_odom);
  rear_lidar_topic_.init(node);
}
#endif

}  // namespace bridge
}  // namespace a2
