#ifndef A2_BRIDGE_CMD_EGRESS_BASE_H_
#define A2_BRIDGE_CMD_EGRESS_BASE_H_

namespace a2 {
namespace bridge {

/**
 * @class EgressType
 * @brief ROS Subscription to DDS publish (ROS to Unitree)
 *
 * Removes all move and assignment constructors to enforce singleton behavior.
 *
 * Implementations are different between robot/ and sim/
 *
 */
struct EgressType {
  EgressType() = default;
  EgressType(EgressType&&) = delete;
  EgressType& operator=(EgressType&&) = delete;
  EgressType(const EgressType&) = delete;
  EgressType& operator=(const EgressType&) = delete;
};

}  // namespace bridge
}  // namespace a2

#endif /* A2_BRIDGE_CMD_EGRESS_BASE_H_ */
