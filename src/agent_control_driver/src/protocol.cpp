#include "agent_control_driver/protocol.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <vector>

namespace agent_control_driver
{
namespace
{

bool parse_double(const std::string & token, double & value)
{
    try {
        std::size_t used = 0;
        value = std::stod(token, &used);
        if (used != token.size() || !std::isfinite(value)) {
            return false;
        }
        return true;
    } catch (const std::exception &) {
        return false;
    }
}

bool parse_int(const std::string & token, int & value)
{
    try {
        std::size_t used = 0;
        const long parsed = std::stol(token, &used);
        if (used != token.size() || parsed < 0 || parsed > 2147483647L) {
            return false;
        }
        value = static_cast<int>(parsed);
        return true;
    } catch (const std::exception &) {
        return false;
    }
}

std::vector<std::string> split(const std::string & line)
{
    std::istringstream stream(line);
    std::vector<std::string> tokens;
    std::string token;
    while (stream >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

ParsedCommand make_error(const std::string & message)
{
    ParsedCommand command;
    command.kind = CommandKind::INVALID;
    command.error = message;
    return command;
}

}  // namespace

bool is_valid_request_id(const std::string & request_id)
{
    if (request_id.empty() || request_id.size() > 64) {
        return false;
    }
    return std::all_of(
        request_id.begin(), request_id.end(),
        [](unsigned char ch) {
            return std::isalnum(ch) != 0 || ch == '_' || ch == '-';
        });
}

ParsedCommand parse_command(const std::string & raw_line)
{
    std::string line = raw_line;
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
        line.pop_back();
    }
    if (line.empty()) {
        return make_error("empty line");
    }
    if (line.size() > 512) {
        return make_error("line too long");
    }

    const std::vector<std::string> tokens = split(line);
    if (tokens.empty()) {
        return make_error("empty line");
    }

    if (tokens[0] == "PING") {
        if (tokens.size() != 1) {
            return make_error("PING takes no arguments");
        }
        ParsedCommand command;
        command.kind = CommandKind::PING;
        return command;
    }

    if (tokens.size() < 2 || !is_valid_request_id(tokens[1])) {
        return make_error("invalid request_id");
    }

    ParsedCommand command;
    command.request_id = tokens[1];

    if (tokens[0] == "MOVE") {
        if (tokens.size() != 7) {
            return make_error("MOVE needs forward, strafe, scale, timeout_ms, lease_ms");
        }
        if (!parse_double(tokens[2], command.forward_m) ||
            !parse_double(tokens[3], command.strafe_m) ||
            !parse_double(tokens[4], command.max_speed_scale) ||
            !parse_int(tokens[5], command.timeout_ms) ||
            !parse_int(tokens[6], command.lease_ms))
        {
            return make_error("MOVE has invalid numeric arguments");
        }
        command.kind = CommandKind::MOVE;
        return command;
    }

    if (tokens[0] == "TURN") {
        if (tokens.size() != 6) {
            return make_error("TURN needs angle, scale, timeout_ms, lease_ms");
        }
        if (!parse_double(tokens[2], command.angle_rad) ||
            !parse_double(tokens[3], command.max_speed_scale) ||
            !parse_int(tokens[4], command.timeout_ms) ||
            !parse_int(tokens[5], command.lease_ms))
        {
            return make_error("TURN has invalid numeric arguments");
        }
        command.kind = CommandKind::TURN;
        return command;
    }

    if (tokens[0] == "CMD") {
        if (tokens.size() != 3 || !parse_int(tokens[2], command.cmd_code)) {
            return make_error("CMD needs a command code");
        }
        command.kind = CommandKind::CMD;
        return command;
    }

    if (tokens[0] == "STOP" || tokens[0] == "ESTOP" || tokens[0] == "ECLEAR" ||
        tokens[0] == "KEEP")
    {
        if (tokens.size() != 2) {
            return make_error(tokens[0] + " takes only request_id");
        }
        if (tokens[0] == "STOP") {
            command.kind = CommandKind::STOP;
        } else if (tokens[0] == "ESTOP") {
            command.kind = CommandKind::ESTOP;
        } else if (tokens[0] == "ECLEAR") {
            command.kind = CommandKind::ECLEAR;
        } else {
            command.kind = CommandKind::KEEP;
        }
        return command;
    }

    return make_error("unknown command");
}

const char * command_kind_name(CommandKind kind)
{
    switch (kind) {
        case CommandKind::PING: return "ping";
        case CommandKind::MOVE: return "move";
        case CommandKind::TURN: return "turn";
        case CommandKind::CMD: return "command";
        case CommandKind::STOP: return "stop";
        case CommandKind::ESTOP: return "estop";
        case CommandKind::ECLEAR: return "estop_clear";
        case CommandKind::KEEP: return "keep";
        case CommandKind::INVALID: return "invalid";
    }
    return "invalid";
}

}  // namespace agent_control_driver
