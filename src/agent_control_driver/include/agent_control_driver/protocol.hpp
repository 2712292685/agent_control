#ifndef AGENT_CONTROL_DRIVER__PROTOCOL_HPP_
#define AGENT_CONTROL_DRIVER__PROTOCOL_HPP_

#include <string>

namespace agent_control_driver
{

enum class CommandKind
{
    INVALID = 0,
    PING,
    MOVE,
    TURN,
    CMD,
    STOP,
    ESTOP,
    ECLEAR,
    KEEP,
};

struct ParsedCommand
{
    CommandKind kind{CommandKind::INVALID};
    std::string request_id;
    double forward_m{0.0};
    double strafe_m{0.0};
    double angle_rad{0.0};
    double max_speed_scale{0.0};
    int timeout_ms{0};
    int lease_ms{0};
    int cmd_code{0};
    std::string error;
};

bool is_valid_request_id(const std::string & request_id);
ParsedCommand parse_command(const std::string & line);
const char * command_kind_name(CommandKind kind);

}  // namespace agent_control_driver

#endif  // AGENT_CONTROL_DRIVER__PROTOCOL_HPP_
