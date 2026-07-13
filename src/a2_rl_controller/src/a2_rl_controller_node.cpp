#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <a2_interfaces/msg/operating_mode.hpp>
#include <vector>
#include <chrono>
#include <mutex>

using namespace std::chrono_literals;

class A2RLControllerNode : public rclcpp::Node
{
public:
  A2RLControllerNode()
  : Node("a2_rl_controller")
  {
    // --- Parámetros ---
    this->declare_parameter<double>("kp", 20.0);
    this->declare_parameter<double>("kd", 5.0);
    this->declare_parameter<bool>("static_hold", true);
    this->declare_parameter<double>("update_rate_hz", 50.0);

    double kp = this->get_parameter("kp").as_double();
    double kd = this->get_parameter("kd").as_double();
    bool static_hold = this->get_parameter("static_hold").as_bool();
    double rate_hz = this->get_parameter("update_rate_hz").as_double();

    RCLCPP_INFO(this->get_logger(), "Params: kp=%.1f, kd=%.1f, static_hold=%d, rate=%.1f Hz",
                kp, kd, static_hold, rate_hz);

    // --- Publicador de comandos de bajo nivel ---
    low_cmd_pub_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

    // --- Suscriptor al estado de las articulaciones ---
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&A2RLControllerNode::joint_state_callback, this, std::placeholders::_1));

    // --- (Opcional) Suscriptor a velocidad base para futuros comandos ---
    // vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    //   "/base_velocity", 10,
    //   std::bind(&A2RLControllerNode::velocity_callback, this, std::placeholders::_1));

    // --- Timer para publicar comandos ---
    timer_ = this->create_wall_timer(
      1s / rate_hz,
      std::bind(&A2RLControllerNode::control_loop, this));

    // --- Inicializar la posición objetivo ---
    target_positions_.resize(12, 0.0);
    positions_received_ = false;

    RCLCPP_INFO(this->get_logger(), "Controller node started.");
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Guardamos las posiciones actuales para mantener la postura
    std::lock_guard<std::mutex> lock(mutex_);
    current_positions_.resize(msg->position.size());
    for (size_t i = 0; i < msg->position.size(); ++i) {
      current_positions_[i] = msg->position[i];
    }

    // Si es la primera vez que recibimos datos, fijamos la posición objetivo
    if (!positions_received_) {
      for (size_t i = 0; i < 12 && i < current_positions_.size(); ++i) {
        target_positions_[i] = current_positions_[i];
      }
      positions_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Initial joint positions captured: [%.3f, %.3f, %.3f, ...]",
                  target_positions_[0], target_positions_[1], target_positions_[2]);
    }
  }

  void control_loop()
  {
    if (!positions_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for joint states...");
      return;
    }

    // Crear mensaje LowCmd
    unitree_go::msg::LowCmd cmd;
    cmd.head[0] = 0xFE;
    cmd.head[1] = 0xEF;
    cmd.level_flag = 0xFF;
    cmd.gpio = 0;

    double kp = this->get_parameter("kp").as_double();
    double kd = this->get_parameter("kd").as_double();
    bool static_hold = this->get_parameter("static_hold").as_bool();

    // Llenar los primeros 12 motores (los del A2)
    for (int i = 0; i < 12; ++i) {
      // Modo 1 = posición, 2 = velocidad, 3 = torque (ver documentación)
      cmd.motor_cmd[i].mode = 1;  // posición
      cmd.motor_cmd[i].q = static_hold ? target_positions_[i] : current_positions_[i];
      cmd.motor_cmd[i].dq = 0.0;
      cmd.motor_cmd[i].tau = 0.0;
      cmd.motor_cmd[i].kp = kp;
      cmd.motor_cmd[i].kd = kd;
    }

    // Los motores restantes (16-19) los ponemos en modo 0 (desactivado)
    for (int i = 12; i < 20; ++i) {
      cmd.motor_cmd[i].mode = 0;
      cmd.motor_cmd[i].q = 0.0;
      cmd.motor_cmd[i].dq = 0.0;
      cmd.motor_cmd[i].tau = 0.0;
      cmd.motor_cmd[i].kp = 0.0;
      cmd.motor_cmd[i].kd = 0.0;
    }

    low_cmd_pub_->publish(cmd);
  }

  // Miembros
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<double> current_positions_;
  std::vector<double> target_positions_;
  bool positions_received_;
  std::mutex mutex_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<A2RLControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}