#include "agent_control_driver/json_util.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace agent_control_driver
{

std::string json_escape(const std::string & text)
{
    std::string out;
    out.reserve(text.size() + 8);
    for (const unsigned char ch : text) {
        switch (ch) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (ch < 0x20) {
                    std::ostringstream escaped;
                    escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                            << static_cast<int>(ch);
                    out += escaped.str();
                } else {
                    out.push_back(static_cast<char>(ch));
                }
        }
    }
    return out;
}

std::string json_number(double value, int precision)
{
    if (!std::isfinite(value)) {
        return "0";
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

std::string json_bool(bool value)
{
    return value ? "true" : "false";
}

}  // namespace agent_control_driver
