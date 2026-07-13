/*
 * Shared converters — work with any LowState type (unitree_hg or unitree_go)
 * that exposes motor_state() / imu_state() with the same field names.
 * sport_mode() uses unitree_go::SportModeState_ for both sim and robot.
 */
#ifndef A2_BRIDGE_COMMON_CONVERTERS_H_
#define A2_BRIDGE_COMMON_CONVERTERS_H_

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

namespace a2 {
namespace bridge {
namespace converters {

inline const std::vector<std::string> kJointNames = {
  "FR_hip_joint",   "FR_thigh_joint", "FR_calf_joint",  "FL_hip_joint",
  "FL_thigh_joint", "FL_calf_joint",  "RR_hip_joint",   "RR_thigh_joint",
  "RR_calf_joint",  "RL_hip_joint",   "RL_thigh_joint", "RL_calf_joint"};

template <typename LowState_t>
inline sensor_msgs::msg::JointState joint_state(const LowState_t& msg,
                                                const builtin_interfaces::msg::Time& stamp) {
  sensor_msgs::msg::JointState ros_msg;
  ros_msg.header.stamp = stamp;
  ros_msg.name = kJointNames;
  for (size_t i = 0; i < kJointNames.size(); ++i) {
    ros_msg.position.push_back(msg.motor_state()[i].q());
    ros_msg.velocity.push_back(msg.motor_state()[i].dq());
    ros_msg.effort.push_back(msg.motor_state()[i].tau_est());
  }
  return ros_msg;
}

template <typename LowState_t>
inline sensor_msgs::msg::Imu imu(const LowState_t& msg,
                                 const builtin_interfaces::msg::Time& stamp) {
  sensor_msgs::msg::Imu ros_msg;
  ros_msg.header.stamp = stamp;
  ros_msg.header.frame_id = "imu_link";
  ros_msg.orientation.w = msg.imu_state().quaternion()[0];
  ros_msg.orientation.x = msg.imu_state().quaternion()[1];
  ros_msg.orientation.y = msg.imu_state().quaternion()[2];
  ros_msg.orientation.z = msg.imu_state().quaternion()[3];
  ros_msg.angular_velocity.x = msg.imu_state().gyroscope()[0];
  ros_msg.angular_velocity.y = msg.imu_state().gyroscope()[1];
  ros_msg.angular_velocity.z = msg.imu_state().gyroscope()[2];
  ros_msg.linear_acceleration.x = msg.imu_state().accelerometer()[0];
  ros_msg.linear_acceleration.y = msg.imu_state().accelerometer()[1];
  ros_msg.linear_acceleration.z = msg.imu_state().accelerometer()[2];
  return ros_msg;
}

// Body IMU rotated to the front lidar IMU frame + rigid-body accel correction.
// R_base_to_lidar = [[0,0,1],[1,0,0],[0,1,0]], so R^T * v = [v.y, v.z, v.x].
// Accelerometer: a_at_r = a_body + ω×(ω×r) + α×r, then rotated by R^T.
template <typename LowState_t>
inline sensor_msgs::msg::Imu front_lidar_imu(const LowState_t& msg,
                                              const builtin_interfaces::msg::Time& stamp,
                                              float prev_gyro[3],
                                              rclcpp::Time& prev_time,
                                              const rclcpp::Time& cur_time) {
  static constexpr float r[3] = {0.34629f, -0.00914f, 0.04218f};

  float gx = msg.imu_state().gyroscope()[0];
  float gy = msg.imu_state().gyroscope()[1];
  float gz = msg.imu_state().gyroscope()[2];

  float dt = static_cast<float>((cur_time - prev_time).seconds());
  float alpha[3] = {0.f, 0.f, 0.f};
  if (dt > 0.f && dt < 0.1f) {
    alpha[0] = (gx - prev_gyro[0]) / dt;
    alpha[1] = (gy - prev_gyro[1]) / dt;
    alpha[2] = (gz - prev_gyro[2]) / dt;
  }
  prev_gyro[0] = gx; prev_gyro[1] = gy; prev_gyro[2] = gz;
  prev_time = cur_time;

  float wxr[3] = { gy*r[2] - gz*r[1], gz*r[0] - gx*r[2], gx*r[1] - gy*r[0] };
  float cent[3] = { gy*wxr[2] - gz*wxr[1], gz*wxr[0] - gx*wxr[2], gx*wxr[1] - gy*wxr[0] };
  float tang[3] = { alpha[1]*r[2] - alpha[2]*r[1],
                     alpha[2]*r[0] - alpha[0]*r[2],
                     alpha[0]*r[1] - alpha[1]*r[0] };

  float ab[3] = { msg.imu_state().accelerometer()[0] + cent[0] + tang[0],
                   msg.imu_state().accelerometer()[1] + cent[1] + tang[1],
                   msg.imu_state().accelerometer()[2] + cent[2] + tang[2] };

  float qw = msg.imu_state().quaternion()[0];
  float qx = msg.imu_state().quaternion()[1];
  float qy = msg.imu_state().quaternion()[2];
  float qz = msg.imu_state().quaternion()[3];

  sensor_msgs::msg::Imu ros_msg;
  ros_msg.header.stamp = stamp;
  ros_msg.header.frame_id = "front_lidar_imu_link";
  ros_msg.angular_velocity.x = gy;
  ros_msg.angular_velocity.y = gz;
  ros_msg.angular_velocity.z = gx;
  ros_msg.linear_acceleration.x = ab[1];
  ros_msg.linear_acceleration.y = ab[2];
  ros_msg.linear_acceleration.z = ab[0];
  ros_msg.orientation.w = 0.5f * (qw - qx - qy - qz);
  ros_msg.orientation.x = 0.5f * (qw + qx + qy - qz);
  ros_msg.orientation.y = 0.5f * (qw - qx + qy + qz);
  ros_msg.orientation.z = 0.5f * (qw + qx - qy + qz);
  return ros_msg;
}

inline std_msgs::msg::UInt8 sport_mode(const unitree_go::msg::dds_::SportModeState_& msg) {
  std_msgs::msg::UInt8 ros_msg;
  ros_msg.data = static_cast<uint8_t>(msg.mode());
  return ros_msg;
}

}  // namespace converters
}  // namespace bridge
}  // namespace a2

#endif /* A2_BRIDGE_COMMON_CONVERTERS_H_ */
