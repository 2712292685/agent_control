#ifndef AGENT_CONTROL_DRIVER__JSON_UTIL_HPP_
#define AGENT_CONTROL_DRIVER__JSON_UTIL_HPP_

#include <string>

namespace agent_control_driver
{

std::string json_escape(const std::string & text);
std::string json_number(double value, int precision = 3);
std::string json_bool(bool value);

}  // namespace agent_control_driver

#endif  // AGENT_CONTROL_DRIVER__JSON_UTIL_HPP_
