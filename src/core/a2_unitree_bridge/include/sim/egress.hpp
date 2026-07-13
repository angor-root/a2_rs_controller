#ifndef A2_BRIDGE_SIM_EGRESS_H_
#define A2_BRIDGE_SIM_EGRESS_H_

#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include "common/egress.hpp"
#include "sim/converters.hpp"

namespace a2 {
namespace bridge {

static const rclcpp::QoS kLowCmdRosQoS{50};

// ─── LowCmdTopic Publisher ───────────────────────────────────────────────────
// ROS publisher sends lowcmd ros messages, convert and publish as lowcmd DDS
struct LowCmdTopic : EgressType {
  using LowCmdDds_t = unitree_go::msg::dds_::LowCmd_;
  using LowCmdRos_t = unitree_go::msg::LowCmd;
  static constexpr const char* dds_topic = "rt/lowcmd";
  static constexpr const char* ros_topic = "/lowcmd";

  unitree::robot::ChannelPublisherPtr<LowCmdDds_t> pub;
  rclcpp::Subscription<LowCmdRos_t>::SharedPtr sub;

  void init(rclcpp::Node* node, const rclcpp::SubscriptionOptions& sub_options) {
    pub.reset(new unitree::robot::ChannelPublisher<LowCmdDds_t>(dds_topic));
    pub->InitChannel();

    sub = node->create_subscription<LowCmdRos_t>(
      ros_topic, kLowCmdRosQoS,
      [this](const LowCmdRos_t::SharedPtr msg) { pub->Write(converters::lowcmd_ros_to_dds(*msg)); },
      sub_options);
  }
};

}  // namespace bridge
}  // namespace a2

#endif
