#ifndef A2_BRIDGE_ROBOT_CMD_EGRESS_H_
#define A2_BRIDGE_ROBOT_CMD_EGRESS_H_

#include <unitree/robot/a2/sport/sport_client.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include "a2_utils/safe_velocity_ros_interface.hpp"
#include "common/egress.hpp"

namespace a2 {
namespace bridge {

/*
 * Wraps the Unitree SportClient SDK. Inherits ROS subscriptions and FSM from
 * SafeVelocityRosInterface and implements onControl() to translate mode into
 * SDK calls.
 */
class A2CommandPublisher : public utils::SafeVelocityRosInterface, public EgressType {
public:
  A2CommandPublisher();

  void init(rclcpp::Node* node);

protected:
  void onControl(utils::OpMode mode, bool mode_changed, std::array<float, 3> vel) override;

private:
  unitree::robot::a2::SportClient sport_client_;
  // rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr debug_vel_pub_;
  // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr debug_hz_pub_;
  // // 0=ESTOP 1=STAND_DOWN 2=STAND_UP 3=BALANCE_STAND 4=VELOCITY_MOVE 5=FREE
  // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr debug_mode_pub_;
  // rclcpp::Time last_vel_time_;
};

}  // namespace bridge
}  // namespace a2

#endif /* A2_BRIDGE_ROBOT_CMD_EGRESS_H_ */
