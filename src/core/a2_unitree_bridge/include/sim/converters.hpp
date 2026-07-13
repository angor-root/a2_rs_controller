/*
 * Sim-specific converters (MuJoCo / unitree_go LowState).
 * Extends the shared converters with odometry, camera, and tick timestamp.
 */
#ifndef A2_BRIDGE_SIM_CONVERTERS_H_
#define A2_BRIDGE_SIM_CONVERTERS_H_

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/idl/ros2/Time_.hpp>
#include "common/converters.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "unitree_go/msg/low_cmd.hpp"

namespace a2 {
namespace bridge {
namespace converters {

inline builtin_interfaces::msg::Time tick_to_stamp(const uint32_t tick) {
  uint64_t ns = static_cast<uint64_t>(tick) * 1000000ULL;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(ns / 1000000000ULL);
  stamp.nanosec = static_cast<uint32_t>(ns % 1000000000ULL);
  return stamp;
}

inline builtin_interfaces::msg::Time stamp(const builtin_interfaces::msg::dds_::Time_& msg) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = msg.sec();
  stamp.nanosec = msg.nanosec();
  return stamp;
}

inline unitree_go::msg::dds_::LowCmd_ lowcmd_ros_to_dds(const unitree_go::msg::LowCmd& ros_msg) {
  // NOTE: Provides bare minimum converter used by unitree_mujoco and a2_locomotion_controller
  // in simulation. This is only motor_cmd and crc
  unitree_go::msg::dds_::LowCmd_ dds_msg;

  std::array<unitree_go::msg::dds_::MotorCmd_, 20> motor_cmds;
  for (size_t i = 0; i < ros_msg.motor_cmd.size(); ++i) {
    unitree_go::msg::dds_::MotorCmd_ cmd;
    cmd.mode(ros_msg.motor_cmd[i].mode);
    cmd.q(ros_msg.motor_cmd[i].q);
    cmd.dq(ros_msg.motor_cmd[i].dq);
    cmd.tau(ros_msg.motor_cmd[i].tau);
    cmd.kp(ros_msg.motor_cmd[i].kp);
    cmd.kd(ros_msg.motor_cmd[i].kd);
    motor_cmds[i] = cmd;
  }
  dds_msg.motor_cmd(motor_cmds);
  dds_msg.crc(ros_msg.crc);

  return dds_msg;
}

inline nav_msgs::msg::Odometry odometry(const unitree_go::msg::dds_::SportModeState_& msg,
                                        const float quat[4], const float ang_vel[3],
                                        const builtin_interfaces::msg::Time& stamp) {
  nav_msgs::msg::Odometry ros_msg;
  ros_msg.header.stamp = stamp;
  ros_msg.header.frame_id = "map";
  ros_msg.child_frame_id = "base_link";
  ros_msg.pose.pose.position.x = msg.position()[0];
  ros_msg.pose.pose.position.y = msg.position()[1];
  ros_msg.pose.pose.position.z = msg.position()[2];
  ros_msg.pose.pose.orientation.w = quat[0];
  ros_msg.pose.pose.orientation.x = quat[1];
  ros_msg.pose.pose.orientation.y = quat[2];
  ros_msg.pose.pose.orientation.z = quat[3];
  ros_msg.twist.twist.linear.x = msg.velocity()[0];
  ros_msg.twist.twist.linear.y = msg.velocity()[1];
  ros_msg.twist.twist.linear.z = msg.velocity()[2];
  ros_msg.twist.twist.angular.x = ang_vel[0];
  ros_msg.twist.twist.angular.y = ang_vel[1];
  ros_msg.twist.twist.angular.z = ang_vel[2];
  return ros_msg;
}

// MuJoCo encodes the camera RGB image inside a PointCloud2 message.
inline sensor_msgs::msg::Image camera_image(const sensor_msgs::msg::dds_::PointCloud2_& msg) {
  sensor_msgs::msg::Image ros_msg;
  ros_msg.header.stamp = stamp(msg.header().stamp());
  ros_msg.header.frame_id = "front_camera_optical_frame";
  ros_msg.height = msg.height();
  ros_msg.width = msg.width();
  ros_msg.encoding = "rgb8";
  ros_msg.is_bigendian = false;
  ros_msg.step = msg.row_step();
  ros_msg.data = msg.data();
  return ros_msg;
}

// CameraInfo intrinsics are no longer hardcoded here — a2_bridge_sim loads them
// from a2_description/config/camera_info_sim.yaml via camera_info_manager
// (see SimCameraTopic in sim/ingress.hpp).

inline sensor_msgs::msg::PointCloud2 pointcloud(const sensor_msgs::msg::dds_::PointCloud2_& msg) {
  sensor_msgs::msg::PointCloud2 ros_msg;
  ros_msg.header.stamp = stamp(msg.header().stamp());
  ros_msg.header.frame_id = msg.header().frame_id();
  ros_msg.height = msg.height();
  ros_msg.width = msg.width();
  ros_msg.is_dense = msg.is_dense();
  ros_msg.is_bigendian = msg.is_bigendian();
  ros_msg.point_step = msg.point_step();
  ros_msg.row_step = msg.row_step();
  ros_msg.data = msg.data();
  for (const auto& field : msg.fields()) {
    sensor_msgs::msg::PointField f;
    f.name = field.name();
    f.offset = field.offset();
    f.datatype = field.datatype();
    f.count = field.count();
    ros_msg.fields.push_back(f);
  }
  return ros_msg;
}

}  // namespace converters
}  // namespace bridge
}  // namespace a2

#endif /* A2_BRIDGE_SIM_CONVERTERS_H_ */
