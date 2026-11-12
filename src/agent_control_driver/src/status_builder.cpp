#include "agent_control_driver/status_builder.hpp"

#include <sstream>

#include "agent_control_driver/json_util.hpp"

namespace agent_control_driver
{
namespace
{

void append_job(std::ostringstream & out, const JobSnapshot & job)
{
    if (!job.present) {
        out << "null";
        return;
    }
    out << "{";
    out << "\"request_id\":\"" << json_escape(job.request_id) << "\"";
    out << ",\"kind\":\"" << json_escape(job.kind) << "\"";
    out << ",\"phase\":\"" << json_escape(job.phase) << "\"";
    out << ",\"state\":\"" << json_escape(job.state) << "\"";
    out << ",\"active\":" << json_bool(job.active);
    out << ",\"requested\":{\"forward_m\":" << json_number(job.requested_forward_m)
        << ",\"strafe_m\":" << json_number(job.requested_strafe_m)
        << ",\"angle_deg\":" << json_number(job.requested_angle_deg) << "}";
    out << ",\"measured\":{\"forward_m\":" << json_number(job.measured_forward_m)
        << ",\"strafe_m\":" << json_number(job.measured_strafe_m)
        << ",\"yaw_deg\":" << json_number(job.measured_yaw_deg) << "}";
    out << ",\"elapsed_ms\":" << json_number(job.elapsed_ms, 0);
    out << ",\"lease_remaining_ms\":" << job.lease_remaining_ms;
    out << ",\"error\":";
    if (job.code.empty() && job.message.empty()) {
        out << "null";
    } else {
        out << "{\"code\":\"" << json_escape(job.code) << "\",\"message\":\""
            << json_escape(job.message) << "\"}";
    }
    out << "}";
}

}  // namespace

std::string build_status_json(const StatusSnapshot & s)
{
    std::ostringstream out;
    out << "{";
    out << "\"schema_version\":1";
    out << ",\"seq\":" << s.seq;
    out << ",\"link_ok\":" << json_bool(s.link_ok);
    out << ",\"executor_ready\":" << json_bool(s.executor_ready);
    out << ",\"connected\":" << json_bool(s.connected);
    out << ",\"in_control\":" << json_bool(s.in_control);
    out << ",\"estop_latched\":" << json_bool(s.estop_latched);
    out << ",\"estop_clear_pending\":" << json_bool(s.estop_clear_pending);
    out << ",\"motion_ready\":" << json_bool(s.motion_ready);
    out << ",\"motion_ready_reason\":\"" << json_escape(s.motion_ready_reason) << "\"";
    out << ",\"odom_ready\":" << json_bool(s.odom_ready);
    out << ",\"role\":\"" << json_escape(s.role) << "\"";
    out << ",\"model\":\"" << json_escape(s.model) << "\"";
    out << ",\"function_mode\":" << s.function_mode;
    out << ",\"function_mode_name\":\"" << json_escape(s.function_mode_name) << "\"";
    out << ",\"control_mode\":" << s.control_mode;
    out << ",\"control_mode_name\":\"" << json_escape(s.control_mode_name) << "\"";
    out << ",\"motion_mode\":" << s.motion_mode;
    out << ",\"motion_mode_name\":\"" << json_escape(s.motion_mode_name) << "\"";
    out << ",\"motion_type\":" << s.motion_type;
    out << ",\"motion_type_name\":\"" << json_escape(s.motion_type_name) << "\"";
    out << ",\"battery\":{\"power\":" << json_number(s.battery.power, 0)
        << ",\"volt\":" << json_number(s.battery.volt, 2)
        << ",\"current\":" << json_number(s.battery.current, 2)
        << ",\"temp\":" << json_number(s.battery.temp, 1)
        << ",\"error\":" << s.battery.error << "}";
    out << ",\"body_temp_c\":" << json_number(s.body_temp_c, 1);
    out << ",\"speed_level\":" << s.speed_level;
    out << ",\"speed_level_name\":\"" << json_escape(s.speed_level_name) << "\"";
    out << ",\"axis_map\":{\"forward\":0,\"strafe\":1,\"yaw\":2,\"head\":3}";
    out << ",\"pose\":{";
    out << "\"position\":[" << json_number(s.position_x) << "," << json_number(s.position_y)
        << "," << json_number(s.position_z) << "]";
    out << ",\"rpy\":[" << json_number(s.roll) << "," << json_number(s.pitch) << ","
        << json_number(s.yaw) << "]";
    out << ",\"yaw_unwrapped\":" << json_number(s.yaw_unwrapped);
    out << ",\"body_velocity\":[" << json_number(s.body_vx) << "," << json_number(s.body_vy)
        << "," << json_number(s.body_vz) << "]";
    out << ",\"world_velocity\":[" << json_number(s.world_vx) << ","
        << json_number(s.world_vy) << "," << json_number(s.world_vz) << "]";
    out << "}";
    out << ",\"active_job\":";
    append_job(out, s.active_job);
    out << ",\"last_job\":";
    append_job(out, s.last_job);
    out << ",\"faults\":[";
    for (std::size_t index = 0; index < s.faults.size(); ++index) {
        if (index > 0) {
            out << ",";
        }
        const FaultSnapshot & fault = s.faults[index];
        out << "{\"module\":\"" << json_escape(fault.module) << "\",\"submodule\":\""
            << json_escape(fault.submodule) << "\",\"level\":\"" << json_escape(fault.level)
            << "\",\"info\":\"" << json_escape(fault.info) << "\",\"code\":" << fault.code
            << "}";
    }
    out << "]";
    out << ",\"last_error\":\"" << json_escape(s.last_error) << "\"";
    out << ",\"tx_hz\":" << json_number(s.tx_hz, 2);
    out << ",\"api_rx_hz\":" << json_number(s.api_rx_hz, 2);
    out << "}";
    return out.str();
}

}  // namespace agent_control_driver
