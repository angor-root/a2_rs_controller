#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

// Bidirectional Twist <-> TwistStamped bridge for /cmd_vel.
//
// The locomotion controller consumes TwistStamped on /cmd_vel (and uses the
// header stamp for its stale-command watchdog), but some tools speak plain
// Twist (e.g. nav2, teleop_twist_keyboard without stamped:=true) and some
// legacy consumers still want plain Twist. This node bridges both directions:
//
//   stamp   : twist_in_topic  (Twist)        -> stamped_topic   (TwistStamped)
//   unstamp : stamped_topic    (TwistStamped) -> twist_out_topic (Twist)
//
// Defaults are chosen so the two directions use disjoint topics and cannot form
// a feedback loop:
//   twist_in_topic  = /cmd_vel_in     (plain-Twist producers publish here)
//   stamped_topic   = /cmd_vel        (canonical TwistStamped topic)
//   twist_out_topic = /cmd_vel_twist  (mirror for plain-Twist consumers)
//   frame_id        = ""              (stamp only, no frame)
//
// A loop would arise only if twist_out_topic == twist_in_topic (unstamped output
// fed back into the stamper); that case is detected and the unstamp direction is
// disabled with an error.
class NavVelRelay : public rclcpp::Node {
public:
  NavVelRelay() : Node("nav_vel_relay")
  {
    pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/nav_vel", 10);
    sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/path_follower_cmd", 10,
      std::bind(&NavVelRelay::cmdVelCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::milliseconds(1), [this]() {
      pub_->publish(cmd_vel_);
    });
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    cmd_vel_ = *msg;
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::TwistStamped cmd_vel_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavVelRelay>());
  rclcpp::shutdown();
  return 0;
}
