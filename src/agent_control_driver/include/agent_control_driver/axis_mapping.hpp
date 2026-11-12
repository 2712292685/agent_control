#ifndef AGENT_CONTROL_DRIVER__AXIS_MAPPING_HPP_
#define AGENT_CONTROL_DRIVER__AXIS_MAPPING_HPP_

#include <array>

namespace agent_control_driver
{

/// SDK API axis order, confirmed empirically on the real robot:
///   [0] forward/backward, positive = forward
///   [1] left/right strafe, positive = left
///   [2] in-place yaw, positive = left
///   [3] head angle (unused)
///
/// The example `remote_control.cpp` comments are wrong: a/d physically turn
/// and q/e physically strafe. Keep this order in one place so the driver and
/// tests cannot drift.
inline std::array<float, 4> make_sdk_axes(
    float forward, float strafe, float yaw, float head = 0.0f)
{
    return {forward, strafe, yaw, head};
}

/// Translate the external control_demo order [forward, yaw, strafe, head]
/// into the SDK order [forward, strafe, yaw, head]. The web and ROS interfaces
/// keep the old external order; only the SDK boundary changes.
inline std::array<float, 4> reorder_external_axes(
    float forward, float yaw, float strafe, float head)
{
    return {forward, strafe, yaw, head};
}

}  // namespace agent_control_driver

#endif  // AGENT_CONTROL_DRIVER__AXIS_MAPPING_HPP_
