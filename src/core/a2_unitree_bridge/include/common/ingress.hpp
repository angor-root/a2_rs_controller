#ifndef A2_BRIDGE_INGRESS_BASE_H_
#define A2_BRIDGE_INGRESS_BASE_H_

namespace a2 {
namespace bridge {

/**
 * @class IngressType
 * @brief DDS Subscription to ROS publish (Unitree to ROS)
 *
 * Removes all move and assignment constructors to enforce singleton behavior.
 *
 * Implementations are different between robot/ and sim/
 *
 */
struct IngressType {
  IngressType() = default;
  IngressType(IngressType&&) = delete;
  IngressType& operator=(IngressType&&) = delete;
  IngressType(const IngressType&) = delete;
  IngressType& operator=(const IngressType&) = delete;
};

}  // namespace bridge
}  // namespace a2

#endif  // A2_BRIDGE_INGRESS_BASE_H_
