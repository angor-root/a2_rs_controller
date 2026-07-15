#ifndef A2_BRIDGE_ROBOT_INGRESS_H_
#define A2_BRIDGE_ROBOT_INGRESS_H_

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include "common/converters.hpp"
#include "common/ingress.hpp"

namespace a2 {
namespace bridge {

inline const rclcpp::QoS kDefaultRosQoS{10};
inline constexpr int64_t kDefaultDdsQueueLen{1};

// ─── LowStateTopic ───────────────────────────────────────────────────────────
// Throttled to 200 Hz. Publishes /a2/joint_states and /a2/imu/data.

struct LowStateTopic : IngressType {
  using LowStateDds_t = unitree_hg::msg::dds_::LowState_;
  using JointState_t = sensor_msgs::msg::JointState;
  using ImuState_t = sensor_msgs::msg::Imu;

  static constexpr const char* dds_topic = "rt/lowstate";
  static constexpr const char* joint_states_topic = "/joint_states";
  static constexpr const char* imu_topic = "/imu/data";

  LowStateDds_t state;
  unitree::robot::ChannelSubscriberPtr<LowStateDds_t> sub;
  rclcpp::Publisher<JointState_t>::SharedPtr joint_pub;
  rclcpp::Publisher<ImuState_t>::SharedPtr imu_pub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub;
  rclcpp::Time last_pub{0, 0, RCL_ROS_TIME};

  void init(rclcpp::Node* node) {
    joint_pub =
      node->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic, kDefaultRosQoS);
    imu_pub = node->create_publisher<sensor_msgs::msg::Imu>(imu_topic, kDefaultRosQoS);
    sub.reset(new unitree::robot::ChannelSubscriber<LowStateDds_t>(dds_topic));
    sub->InitChannel(
      [this, node](const void* msg) {
        state = *static_cast<const LowStateDds_t*>(msg);
        builtin_interfaces::msg::Time stamp = node->get_clock()->now();
        rclcpp::Time t(stamp);
        if ((t - last_pub).seconds() < 0.004)
          return;
        last_pub = t;
        joint_pub->publish(converters::joint_state(state, stamp));
        imu_pub->publish(converters::imu(state, stamp));
      },
      kDefaultDdsQueueLen);
  }
};

// ─── SportStateTopic ─────────────────────────────────────────────────────────
// Throttled to 50 Hz. Publishes /a2/sport_mode as UInt8.

struct SportStateTopic : IngressType {
  using DdsTopic_t = unitree_go::msg::dds_::SportModeState_;
  static constexpr const char* dds_topic = "rt/sportmodestate";

  DdsTopic_t state;
  unitree::robot::ChannelSubscriberPtr<DdsTopic_t> sub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_pub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub; 
  rclcpp::Time last_pub{0, 0, RCL_ROS_TIME};

  void init(rclcpp::Node* node) {
    mode_pub = node->create_publisher<std_msgs::msg::UInt8>("/a2/sport_mode", kDefaultRosQoS);
    vel_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/base_velocity", kDefaultRosQoS);
    sub.reset(new unitree::robot::ChannelSubscriber<DdsTopic_t>(dds_topic));
    sub->InitChannel(
      [this, node](const void* msg) {
        RCLCPP_INFO(node->get_logger(), "SportStateTopic callback ENTERED");  // ← LOG 1
        state = *static_cast<const DdsTopic_t*>(msg);
        rclcpp::Time now = node->get_clock()->now();
        if ((now - last_pub).seconds() < 0.02) {
          RCLCPP_INFO(node->get_logger(), "Throttling: skipping publish");   // ← LOG 2
          return;
        }
        last_pub = now;
        RCLCPP_INFO(node->get_logger(), "Publishing sport_mode");           // ← LOG 3
        mode_pub->publish(converters::sport_mode(state));
        
        if (!vel_pub) {
          RCLCPP_ERROR(node->get_logger(), "vel_pub is NULL!");             // ← LOG 4
          return;
        }
        RCLCPP_INFO(node->get_logger(), "Publishing base_velocity");        // ← LOG 5
        geometry_msgs::msg::TwistStamped vel_msg;
        vel_msg.header.stamp = now;
        vel_msg.header.frame_id = "base_link";
        vel_msg.twist.linear.x = state.velocity()[0];
        vel_msg.twist.linear.y = state.velocity()[1];
        vel_msg.twist.linear.z = state.velocity()[2];
        vel_msg.twist.angular.x = 0.0;
        vel_msg.twist.angular.y = 0.0;
        vel_msg.twist.angular.z = state.yaw_speed();
        vel_pub->publish(vel_msg);
        RCLCPP_INFO(node->get_logger(), "base_velocity published");
      },
      kDefaultDdsQueueLen);
  }
};

}  // namespace bridge
}  // namespace a2

#endif
