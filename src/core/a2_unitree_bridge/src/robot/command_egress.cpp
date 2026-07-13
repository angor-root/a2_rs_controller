#include "robot/command_egress.hpp"

#include <chrono>
#include <rclcpp/logging.hpp>

namespace {
// TODO: expose via ROS params
constexpr float kMaxVelX{2.0f}; // m/s
constexpr float kMaxVelY{1.0f}; // m/s
constexpr float kMaxYawRate{2.0f}; // rad/s?
constexpr std::chrono::milliseconds kControlPeriod{1}; // 25 Hz
constexpr int64_t kCmdVelMaxAgeNs{500'000'000LL};  // 500 ms

}  // namespace

namespace a2 {
namespace bridge {

A2CommandPublisher::A2CommandPublisher()
    : SafeVelocityRosInterface(kMaxVelX, kMaxVelY, kMaxYawRate, kControlPeriod, kCmdVelMaxAgeNs) {}

void A2CommandPublisher::init(rclcpp::Node* node) {
  sport_client_.SetTimeout(0.1f);
  sport_client_.Init();
  // debug_vel_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>("/debug/egress_vel", 10);
  // debug_hz_pub_ = node->create_publisher<std_msgs::msg::Float32>("/debug/egress_hz", 10);
  // debug_mode_pub_ = node->create_publisher<std_msgs::msg::Int32>("/debug/egress_mode", 10);
  // last_vel_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  SafeVelocityRosInterface::init(node);
}

void A2CommandPublisher::onControl(utils::OpMode mode, bool mode_changed,
                                   std::array<float, 3> vel) {
  // std_msgs::msg::Int32 mode_msg;
  // mode_msg.data = static_cast<int32_t>(mode);
  // debug_mode_pub_->publish(mode_msg);

  int32_t rc = 0;

  switch (mode) {
    case utils::OpMode::ESTOP: {
      RCLCPP_DEBUG(node_->get_logger(), "Mode: ESTOP/DAMP");
      rc = sport_client_.Damp();
      break;
    }
    case utils::OpMode::STAND_DOWN: {
      RCLCPP_DEBUG(node_->get_logger(), "Mode: STAND_DOWN");
      rc = sport_client_.StandDown();
      break;
    }
    case utils::OpMode::STAND_UP: {
      RCLCPP_DEBUG(node_->get_logger(), "Mode: STAND_UP");
      rc = sport_client_.StandUp();
      break;
    }
    case utils::OpMode::BALANCE_STAND: {
      RCLCPP_DEBUG(node_->get_logger(), "Mode: BALANCE_STAND");
      rc = sport_client_.BalanceStand();
      break;
    }
    case utils::OpMode::VELOCITY_MOVE: {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Mode: VELOCITY_MOVE");
      rc = sport_client_.Move(vel[0], vel[1], vel[2]);

      // auto now = node_->get_clock()->now();
      // geometry_msgs::msg::TwistStamped vel_msg;
      // vel_msg.header.stamp = now;
      // vel_msg.twist.linear.x = vel[0];
      // vel_msg.twist.linear.y = vel[1];
      // vel_msg.twist.angular.z = vel[2];
      // debug_vel_pub_->publish(vel_msg);

      // if (last_vel_time_.nanoseconds() > 0) {
      //   double dt = (now - last_vel_time_).seconds();
      //   if (dt > 0.0) {
      //     std_msgs::msg::Float32 hz_msg;
      //     hz_msg.data = static_cast<float>(1.0 / dt);
      //     debug_hz_pub_->publish(hz_msg);
      //   }
      // }
      // last_vel_time_ = now;
      break;
    }
    case utils::OpMode::FREE: {
      RCLCPP_DEBUG(node_->get_logger(), "Mode: FREE");
      rc = sport_client_.StopMove();
      break;
    }
    default:
      RCLCPP_WARN(node_->get_logger(), "Unknown mode requested, ignoring");
      break;
  }

  if (rc != 0) {
    RCLCPP_WARN(node_->get_logger(), "SportClient error %d (mode=%d)", rc,
                static_cast<int>(mode));
  }
}

}  // namespace bridge
}  // namespace a2
