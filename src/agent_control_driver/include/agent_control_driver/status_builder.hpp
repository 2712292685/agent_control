#ifndef AGENT_CONTROL_DRIVER__STATUS_BUILDER_HPP_
#define AGENT_CONTROL_DRIVER__STATUS_BUILDER_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace agent_control_driver
{

struct BatterySnapshot
{
    double power{0.0};
    double volt{0.0};
    double current{0.0};
    double temp{0.0};
    int error{0};
};

struct FaultSnapshot
{
    std::string module;
    std::string submodule;
    std::string level;
    std::string info;
    int code{0};
};

struct JobSnapshot
{
    bool present{false};
    bool active{false};
    std::string request_id;
    std::string kind;
    std::string phase;
    std::string state;
    double requested_forward_m{0.0};
    double requested_strafe_m{0.0};
    double requested_angle_deg{0.0};
    double measured_forward_m{0.0};
    double measured_strafe_m{0.0};
    double measured_yaw_deg{0.0};
    double elapsed_ms{0.0};
    long lease_remaining_ms{0};
    std::string code;
    std::string message;
};

struct StatusSnapshot
{
    std::uint64_t seq{0};
    bool link_ok{false};
    bool executor_ready{false};
    bool connected{false};
    bool in_control{false};
    bool estop_latched{false};
    bool estop_clear_pending{false};
    bool motion_ready{false};
    bool odom_ready{false};
    std::string motion_ready_reason;
    std::string role;
    std::string model;
    int function_mode{0};
    std::string function_mode_name;
    int control_mode{0};
    std::string control_mode_name;
    int motion_mode{0};
    std::string motion_mode_name;
    int motion_type{0};
    std::string motion_type_name;
    BatterySnapshot battery;
    double body_temp_c{0.0};
    int speed_level{0};
    std::string speed_level_name;
    double position_x{0.0};
    double position_y{0.0};
    double position_z{0.0};
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
    double yaw_unwrapped{0.0};
    double body_vx{0.0};
    double body_vy{0.0};
    double body_vz{0.0};
    double world_vx{0.0};
    double world_vy{0.0};
    double world_vz{0.0};
    std::vector<FaultSnapshot> faults;
    JobSnapshot active_job;
    JobSnapshot last_job;
    std::string last_error;
    double tx_hz{0.0};
    double api_rx_hz{0.0};
};

std::string build_status_json(const StatusSnapshot & snapshot);

}  // namespace agent_control_driver

#endif  // AGENT_CONTROL_DRIVER__STATUS_BUILDER_HPP_
