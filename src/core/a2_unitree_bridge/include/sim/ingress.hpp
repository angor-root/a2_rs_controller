#ifndef A2_BRIDGE_SIM_INGRESS_H_
#define A2_BRIDGE_SIM_INGRESS_H_

#include "common/ingress.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "builtin_interfaces/msg/time.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sim/converters.hpp"

namespace a2 {
namespace bridge {

static const rclcpp::QoS kDefaultRosQoS{10};
static const int64_t kDefaultDdsQueueLen{1};

// ─── LowStateTopic ───────────────────────────────────────────────────────────
// Publishes sim_clock from mujoco
// Publishes /joint_states and /imu/data. Throttled to 200 Hz.
// Writes to the bridge's shared cache (quat, ang_vel, stamp) so SportStateTopic
// can use an authoritative timestamp and orientation from the same LowState
// tick.

struct LowStateTopic : IngressType {
  using LowStateDds_t = unitree_go::msg::dds_::LowState_;
  using JointState_t = sensor_msgs::msg::JointState;
  using ImuState_t = sensor_msgs::msg::Imu;
  using ClockState_t = rosgraph_msgs::msg::Clock;

  static constexpr const char* dds_topic = "rt/lowstate";
  static constexpr const char* joint_states_topic = "/joint_states";
  static constexpr const char* imu_topic = "/imu/data";
  static constexpr const char* clock_topic = "/clock";

  LowStateDds_t state;
  unitree::robot::ChannelSubscriberPtr<LowStateDds_t> sub;
  rclcpp::Publisher<JointState_t>::SharedPtr joint_pub;
  rclcpp::Publisher<ImuState_t>::SharedPtr imu_pub;
  rclcpp::Publisher<ClockState_t>::SharedPtr clock_pub;
  rclcpp::Time last_pub{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<ImuState_t>::SharedPtr front_lidar_imu_pub;
  float prev_gyro_[3] = {0, 0, 0};
  rclcpp::Time prev_imu_time_{0, 0, RCL_ROS_TIME};

  void init(rclcpp::Node* node, float* cached_quat, float* cached_ang_vel,
            builtin_interfaces::msg::Time* cached_stamp, std::mutex* cache_mutex) {
    joint_pub = node->create_publisher<JointState_t>(joint_states_topic, kDefaultRosQoS);
    imu_pub = node->create_publisher<ImuState_t>(imu_topic, kDefaultRosQoS);
    front_lidar_imu_pub = node->create_publisher<ImuState_t>("/front_lidar/imu", kDefaultRosQoS);
    clock_pub = node->create_publisher<ClockState_t>(clock_topic, kDefaultRosQoS);

    sub.reset(new unitree::robot::ChannelSubscriber<LowStateDds_t>(dds_topic));
    sub->InitChannel(
      [this, cached_quat, cached_ang_vel, cached_stamp, cache_mutex](const void* msg) {
        state = *static_cast<const LowStateDds_t*>(msg);
        builtin_interfaces::msg::Time stamp = converters::tick_to_stamp(state.tick());

        ClockState_t clock_msg;
        clock_msg.clock = stamp;
        clock_pub->publish(clock_msg);

        {
          std::lock_guard<std::mutex> lock(*cache_mutex);
          *cached_stamp = stamp;
          cached_quat[0] = state.imu_state().quaternion()[0];
          cached_quat[1] = state.imu_state().quaternion()[1];
          cached_quat[2] = state.imu_state().quaternion()[2];
          cached_quat[3] = state.imu_state().quaternion()[3];
          cached_ang_vel[0] = state.imu_state().gyroscope()[0];
          cached_ang_vel[1] = state.imu_state().gyroscope()[1];
          cached_ang_vel[2] = state.imu_state().gyroscope()[2];
        }

        rclcpp::Time t(stamp);
        if ((t - last_pub).seconds() < 0.005)
          return;
        last_pub = t;

        joint_pub->publish(converters::joint_state(state, stamp));
        imu_pub->publish(converters::imu(state, stamp));
        front_lidar_imu_pub->publish(
          converters::front_lidar_imu(state, stamp, prev_gyro_, prev_imu_time_, t));
      },
      kDefaultDdsQueueLen);
  }
};

// ─── SportStateTopic ─────────────────────────────────────────────────────────
// Throttled to 50 Hz. Reads cached quat/ang_vel/stamp from LowStateTopic.
// Publishes /odom, /state_estimation, /a2/sport_mode. Broadcasts map→base_link
// TF.

struct SportStateTopic : IngressType {
  using DdsTopic_t = unitree_go::msg::dds_::SportModeState_;
  static constexpr const char* dds_topic = "rt/sportmodestate";
  static constexpr const char* sport_mode_topic = "/a2/sport_mode";
  static constexpr const char* odom_topic = "/odom";
  static constexpr const char* state_est_topic = "/state_estimation";

  DdsTopic_t state;
  unitree::robot::ChannelSubscriberPtr<DdsTopic_t> sub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_est_pub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  rclcpp::Time last_pub{0, 0, RCL_ROS_TIME};
  bool publish_odom_ = true;

  void init(rclcpp::Node* node, const float* cached_quat, const float* cached_ang_vel,
            const builtin_interfaces::msg::Time* cached_stamp, std::mutex* cache_mutex) {
    node->get_parameter_or("publish_odom", publish_odom_, true);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    mode_pub = node->create_publisher<std_msgs::msg::UInt8>(sport_mode_topic, kDefaultRosQoS);
    odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(odom_topic, kDefaultRosQoS);
    state_est_pub =
      node->create_publisher<nav_msgs::msg::Odometry>(state_est_topic, kDefaultRosQoS);

    sub.reset(new unitree::robot::ChannelSubscriber<DdsTopic_t>(dds_topic));
    sub->InitChannel(
      [this, cached_quat, cached_ang_vel, cached_stamp, cache_mutex](const void* msg) {
        state = *static_cast<const DdsTopic_t*>(msg);

        builtin_interfaces::msg::Time stamp;
        float quat[4];
        float ang_vel[3];
        {
          std::lock_guard<std::mutex> lock(*cache_mutex);
          stamp = *cached_stamp;
          quat[0] = cached_quat[0];
          quat[1] = cached_quat[1];
          quat[2] = cached_quat[2];
          quat[3] = cached_quat[3];
          ang_vel[0] = cached_ang_vel[0];
          ang_vel[1] = cached_ang_vel[1];
          ang_vel[2] = cached_ang_vel[2];
        }

        rclcpp::Time t(stamp);
        if ((t - last_pub).seconds() < 0.02)
          return;
        last_pub = t;

        mode_pub->publish(converters::sport_mode(state));

        if (!publish_odom_) return;

        auto odom = converters::odometry(state, quat, ang_vel, stamp);
        odom_pub->publish(odom);
        state_est_pub->publish(odom);

        geometry_msgs::msg::TransformStamped tf;
        tf.header = odom.header;
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = state.position()[0];
        tf.transform.translation.y = state.position()[1];
        tf.transform.translation.z = state.position()[2];
        tf.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster->sendTransform(tf);
      },
      kDefaultDdsQueueLen);
  }
};

// ─── SimCameraTopic ───────────────────────────────────────────────────────────
// MuJoCo encodes the front camera RGB image inside a PointCloud2 DDS message.
// Converts to sensor_msgs::Image + publishes CameraInfo intrinsics loaded from
// a2_description/config/camera_info_sim.yaml (the sim counterpart of the real
// robot's camera_info_real.yaml).
struct SimCameraTopic : IngressType {
  using PointCloudDds_t = sensor_msgs::msg::dds_::PointCloud2_;
  static constexpr const char* dds_topic = "rt/mujoco/front_camera_pointcloud";
  static constexpr const char* image_pub_topic = "/camera/image_raw";
  static constexpr const char* info_pub_topic = "/camera/camera_info";
  static constexpr const char* camera_info_url =
    "package://a2_description/config/camera_info_sim.yaml";
  static constexpr const char* optical_frame = "front_camera_optical_frame";

  unitree::robot::ChannelSubscriberPtr<PointCloudDds_t> sub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_mgr;
  sensor_msgs::msg::CameraInfo cinfo;  // cached intrinsics, stamped per frame

  void init(rclcpp::Node* node) {
    image_pub = node->create_publisher<sensor_msgs::msg::Image>(image_pub_topic, kDefaultRosQoS);
    info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(info_pub_topic, kDefaultRosQoS);

    // Load the sim intrinsics once; only the timestamp changes per frame.
    cinfo_mgr = std::make_shared<camera_info_manager::CameraInfoManager>(node, "front_camera");
    if (!cinfo_mgr->loadCameraInfo(camera_info_url)) {
      RCLCPP_WARN(node->get_logger(),
                  "Failed to load sim camera_info from %s; publishing empty intrinsics",
                  camera_info_url);
    }
    cinfo = cinfo_mgr->getCameraInfo();
    cinfo.header.frame_id = optical_frame;

    sub.reset(new unitree::robot::ChannelSubscriber<PointCloudDds_t>(dds_topic));
    sub->InitChannel(
      [this](const void* msg) {
        auto state = *static_cast<const PointCloudDds_t*>(msg);
        image_pub->publish(converters::camera_image(state));
        cinfo.header.stamp = converters::stamp(state.header().stamp());
        info_pub->publish(cinfo);
      },
      kDefaultDdsQueueLen);
  }
};

// ─── SimLidarTopic ────────────────────────────────────────────────────────────
// Subscribes to a MuJoCo lidar DDS topic and publishes the raw cloud as a ROS
// topic. Optionally (pass with_registered_scan=true for the front lidar) also
// transforms the cloud into the map frame and publishes /registered_scan for
// the navigation stack — all in the same DDS callback, no extra ROS hop.

template <typename Type>
struct SimLidarTopic : IngressType {
  using PointCloudDds_t = sensor_msgs::msg::dds_::PointCloud2_;
  using PointCloudRos_t = sensor_msgs::msg::PointCloud2;

  unitree::robot::ChannelSubscriberPtr<PointCloudDds_t> sub;
  rclcpp::Publisher<PointCloudRos_t>::SharedPtr raw_pub;

  // Populated only when with_registered_scan=true
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  rclcpp::Publisher<PointCloudRos_t>::SharedPtr registered_scan_pub;

  void init(rclcpp::Node* node, bool with_registered_scan = false) {
    raw_pub = node->create_publisher<PointCloudRos_t>(Type::ros_topic, kDefaultRosQoS);

    if (with_registered_scan) {
      tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
      tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node);
      registered_scan_pub =
        node->create_publisher<PointCloudRos_t>("/registered_scan", kDefaultRosQoS);
    }

    sub.reset(new unitree::robot::ChannelSubscriber<PointCloudDds_t>(Type::dds_topic));
    sub->InitChannel(
      [this, node](const void* msg) {
        auto state = *static_cast<const PointCloudDds_t*>(msg);
        PointCloudRos_t raw_cloud = converters::pointcloud(state);
        raw_pub->publish(raw_cloud);

        if (!registered_scan_pub)
          return;

        try {
          // Query 25ms in the past — TF updates at ~50 Hz so this is always available.
          rclcpp::Time t_query =
            rclcpp::Time(raw_cloud.header.stamp) - rclcpp::Duration::from_seconds(0.025);
          auto transform = tf_buffer->lookupTransform("map", raw_cloud.header.frame_id, t_query);

          PointCloudRos_t cloud_map;
          tf2::doTransform(raw_cloud, cloud_map, transform);
          cloud_map.header.stamp = raw_cloud.header.stamp;
          cloud_map.header.frame_id = "map";

          // Add 'intensity' alias for the 'dist' field so terrain_analysis can
          // parse the cloud as PointXYZI without warnings. No data copy.
          uint32_t dist_offset = 12;
          for (const auto& f : cloud_map.fields) {
            if (f.name == "dist") {
              dist_offset = f.offset;
              break;
            }
          }
          PointCloudRos_t::_fields_type::value_type intensity;
          intensity.name = "intensity";
          intensity.offset = dist_offset;
          intensity.datatype = sensor_msgs::msg::PointField::FLOAT32;
          intensity.count = 1;
          cloud_map.fields.push_back(intensity);

          registered_scan_pub->publish(cloud_map);
        } catch (const tf2::TransformException& e) {
          RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                               "Registered scan TF lookup failed: %s", e.what());
        }
      },
      kDefaultDdsQueueLen);
  }
};

struct FrontLidarTraits {
  static constexpr const char* dds_topic = "rt/mujoco/front_lidar";
  static constexpr const char* ros_topic = "/front_lidar/points";
};
using FrontLidarTopic = SimLidarTopic<FrontLidarTraits>;

struct RearLidarTraits {
  static constexpr const char* dds_topic = "rt/mujoco/rear_lidar";
  static constexpr const char* ros_topic = "/rear_lidar/points";
};
using RearLidarTopic = SimLidarTopic<RearLidarTraits>;

}  // namespace bridge
}  // namespace a2

#endif /* A2_BRIDGE_SIM_INGRESS_H_ */
