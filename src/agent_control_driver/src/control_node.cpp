#include "agent_control_driver/control_node.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

#include "agent_control_driver/axis_mapping.hpp"
#include "agent_control_driver/json_util.hpp"
#include "agent_control_driver/protocol.hpp"

namespace agent_control_driver
{
namespace
{

constexpr int64_t kMinPort = 1;
constexpr int64_t kMaxPort = 65535;
constexpr double kPi = 3.14159265358979323846;
constexpr int kEstopResendCount = 3;
constexpr auto kEstopResendPeriod = std::chrono::milliseconds(200);
constexpr auto kRecoveryConfirmDelay = std::chrono::milliseconds(300);
constexpr auto kRecoveryTimeout = std::chrono::seconds(5);
constexpr auto kPostureHold = std::chrono::milliseconds(200);
constexpr auto kPostureTimeout = std::chrono::seconds(5);
constexpr int kShutdownZeroCount = 5;
constexpr auto kShutdownZeroPeriod = std::chrono::milliseconds(10);

constexpr int kCmdEmergencyStop = 90;
constexpr int kCmdStandUp = 122;
constexpr int kCmdSitDown = 106;
constexpr int kCmdMoveMode = 138;
constexpr int kCmdBalanceStand = 154;
constexpr int kCmdLock = 155;
constexpr int kCmdSlow = 174;
constexpr int kCmdSdkControlRight = 179;

rcl_interfaces::msg::ParameterDescriptor make_descriptor(const std::string & description)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return descriptor;
}

std::string to_lower(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return text;
}

template <typename T>
bool in_range(T value, T low, T high)
{
    return value >= low && value <= high;
}

int64_t now_ms_epoch()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string model_name(int value)
{
    switch (static_cast<zsibot::Model>(value)) {
        case zsibot::Model::MODEL_XG: return "XG";
        case zsibot::Model::MODEL_XGW: return "XGW";
        case zsibot::Model::MODEL_XGWHSPD: return "XGWHSPD";
    }
    return "unknown";
}

std::string function_mode_name(int value)
{
    switch (static_cast<zsibot::FunctionMode>(value)) {
        case zsibot::FunctionMode::FM_NULL: return "null";
        case zsibot::FunctionMode::FM_REMOTE: return "remote";
        case zsibot::FunctionMode::FM_TRACE: return "trace";
        case zsibot::FunctionMode::FM_SDK: return "sdk";
        case zsibot::FunctionMode::FM_GENERAL_SDK: return "general_sdk";
        case zsibot::FunctionMode::FM_ROAMER: return "roamer";
    }
    return "unknown";
}

std::string control_mode_name(int value)
{
    switch (static_cast<zsibot::ControlMode>(value)) {
        case zsibot::ControlMode::CM_NULL: return "null";
        case zsibot::ControlMode::CM_STAND_UP: return "stand_up";
        case zsibot::ControlMode::CM_SIT_DOWN: return "stand_down";
        case zsibot::ControlMode::CM_LOCK_MODE: return "lock";
        case zsibot::ControlMode::CM_EMERGENCY_STOP: return "emergency";
        case zsibot::ControlMode::CM_MOVE_MODE: return "move";
        case zsibot::ControlMode::CM_BALANCE_STAND_MODE: return "stay";
        case zsibot::ControlMode::CM_INIT: return "init";
    }
    return "unknown";
}

std::string motion_mode_name(int value)
{
    switch (static_cast<zsibot::MotionMode>(value)) {
        case zsibot::MotionMode::MM_NULL: return "null";
        case zsibot::MotionMode::MM_RUNNING: return "running";
        case zsibot::MotionMode::MM_REST: return "rest";
        case zsibot::MotionMode::MM_FORBID: return "forbid";
    }
    return "unknown";
}

std::string motion_type_name(int value)
{
    switch (static_cast<zsibot::MotionType>(value)) {
        case zsibot::MotionType::MT_NULL: return "null";
        case zsibot::MotionType::MT_JUMP: return "jump";
        case zsibot::MotionType::MT_FORWARD_JUMP: return "forward_jump";
        case zsibot::MotionType::MT_BACK_FLIP: return "back_flip";
        case zsibot::MotionType::MT_GREET: return "greet";
        case zsibot::MotionType::MT_TWO_LEG_STAND: return "two_leg_stand";
        case zsibot::MotionType::MT_FORBID: return "forbid";
        case zsibot::MotionType::MT_UNKNOWN: return "unknown";
        case zsibot::MotionType::MT_UNLOADING_SQUAT: return "unloading_squat";
    }
    return "unknown";
}

std::string speed_level_name(int value)
{
    switch (static_cast<zsibot::SpeedLevel>(value)) {
        case zsibot::SpeedLevel::SL_NULL: return "null";
        case zsibot::SpeedLevel::SL_SLOW: return "slow";
        case zsibot::SpeedLevel::SL_NORMAL: return "normal";
        case zsibot::SpeedLevel::SL_FAST: return "fast";
    }
    return "unknown";
}

}  // namespace

AgentControlNode::AgentControlNode()
: rclcpp::Node("agent_control_driver")
{
    robot_ip_ = declare_parameter<std::string>(
        "robot_ip", "192.168.234.1", make_descriptor("机器狗 IP 地址"));
    send_port_ = declare_parameter<int64_t>("send_port", 8081, make_descriptor("UDP 发送端口"));
    recv_port_ = declare_parameter<int64_t>("recv_port", 8080, make_descriptor("UDP 接收端口"));
    role_ = declare_parameter<std::string>("role", "sdk", make_descriptor("sdk 或 remote"));
    control_rate_hz_ = declare_parameter<double>(
        "control_rate_hz", 30.0, make_descriptor("SetRemote 频率（Hz）"));
    status_rate_hz_ = declare_parameter<double>(
        "status_rate_hz", 5.0, make_descriptor("状态发布频率（Hz）"));
    auto_request_control_ = declare_parameter<bool>(
        "auto_request_control", true, make_descriptor("自动请求 SDK 控制权"));
    control_request_period_sec_ = declare_parameter<double>(
        "control_request_period_sec", 1.0, make_descriptor("控制权请求周期（秒）"));
    max_speed_scale_ = declare_parameter<double>(
        "max_speed_scale", 0.35, make_descriptor("默认最大速度比例，0.05~0.6"));
    bridge_port_ = declare_parameter<int64_t>(
        "bridge_port", 8766, make_descriptor("本机 TCP 控制桥端口"));
    motion_timeout_ms_ = declare_parameter<int64_t>(
        "motion_timeout_ms", 35000, make_descriptor("单个运动任务最大时长（毫秒）"));
    lease_ms_ = declare_parameter<int64_t>(
        "lease_ms", 1500, make_descriptor("驱动运动 lease（毫秒）"));
    api_link_timeout_ms_ = declare_parameter<int64_t>(
        "api_link_timeout_ms", 1000, make_descriptor("API 链路静默超时（毫秒）"));
    position_tolerance_m_ = declare_parameter<double>(
        "position_tolerance_m", 0.03, make_descriptor("停车位置容差（米）"));
    angle_tolerance_deg_ = declare_parameter<double>(
        "angle_tolerance_deg", 2.0, make_descriptor("停车角度容差（度）"));
    acceptance_m_ = declare_parameter<double>(
        "acceptance_m", 0.05, make_descriptor("成功判定位置误差（米）"));
    acceptance_deg_ = declare_parameter<double>(
        "acceptance_deg", 5.0, make_descriptor("成功判定角度误差（度）"));
    no_progress_timeout_ms_ = declare_parameter<int64_t>(
        "no_progress_timeout_ms", 2000, make_descriptor("无进度超时（毫秒）"));
    settle_ms_ = declare_parameter<int64_t>(
        "settle_ms", 600, make_descriptor("停车后 settle 时间（毫秒）"));
    force_slow_speed_ = declare_parameter<bool>(
        "force_slow_speed", true, make_descriptor("取得控制权后发送一次慢速档"));
    sdk_forward_sign_ = declare_parameter<double>(
        "sdk_forward_sign", 1.0, make_descriptor("前进轴符号，1 或 -1"));
    sdk_strafe_sign_ = declare_parameter<double>(
        "sdk_strafe_sign", 1.0, make_descriptor("平移轴符号，1 或 -1"));
    sdk_yaw_sign_ = declare_parameter<double>(
        "sdk_yaw_sign", 1.0, make_descriptor("转向轴符号，1 或 -1"));
    yaw_feedback_sign_ = declare_parameter<double>(
        "yaw_feedback_sign", 1.0, make_descriptor("里程计 yaw 符号，1 或 -1"));
    estop_recovery_cmd_ = declare_parameter<int64_t>(
        "estop_recovery_cmd", 122, make_descriptor("解除急停时发送的姿态命令码"));
    warn_period_sec_ = declare_parameter<double>(
        "warn_period_sec", 5.0, make_descriptor("日志限流周期（秒）"));
    max_forward_m_ = declare_parameter<double>(
        "max_forward_m", 5.0, make_descriptor("单次前后最大距离（米）"));
    max_strafe_m_ = declare_parameter<double>(
        "max_strafe_m", 3.0, make_descriptor("单次平移最大距离（米）"));
    max_turn_deg_ = declare_parameter<double>(
        "max_turn_deg", 180.0, make_descriptor("单次转向最大角度（度）"));

    validate_parameters();

    RCLCPP_INFO(
        get_logger(),
        "启动: robot_ip=%s send=%lld recv=%lld role=%s rate=%.1fHz "
        "bridge=127.0.0.1:%lld max_speed=%.2f force_slow=%s",
        robot_ip_.c_str(), static_cast<long long>(send_port_),
        static_cast<long long>(recv_port_), role_.c_str(), control_rate_hz_,
        static_cast<long long>(bridge_port_), max_speed_scale_,
        force_slow_speed_ ? "true" : "false");

    holder_ = std::make_shared<ExecutorHolder>();
    holder_->ensure(
        parse_role(), robot_ip_, static_cast<uint32_t>(send_port_),
        static_cast<uint32_t>(recv_port_), std::chrono::steady_clock::now());

    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/agent_control/status", rclcpp::QoS(1).transient_local());

    bridge_thread_ = std::thread(&AgentControlNode::bridge_server_loop, this);

    const auto period = std::chrono::milliseconds(
        std::max<int64_t>(1, static_cast<int64_t>(1000.0 / control_rate_hz_)));
    control_timer_ = create_wall_timer(
        period, std::bind(&AgentControlNode::on_control_tick, this));
}

AgentControlNode::~AgentControlNode()
{
    if (control_timer_) {
        control_timer_->cancel();
    }
    bridge_stop_ = true;
    if (bridge_thread_.joinable()) {
        bridge_thread_.join();
    }
    shutdown_motion();
    holder_->shutdown();
}

void AgentControlNode::validate_parameters()
{
    if (robot_ip_.empty() ||
        std::any_of(robot_ip_.begin(), robot_ip_.end(), [](unsigned char ch) {
            return std::isspace(ch) != 0;
        }))
    {
        throw std::invalid_argument("robot_ip 不能为空或包含空白");
    }
    if (!in_range<int64_t>(send_port_, kMinPort, kMaxPort) ||
        !in_range<int64_t>(recv_port_, kMinPort, kMaxPort) ||
        send_port_ == recv_port_)
    {
        throw std::invalid_argument("send_port/recv_port 无效或相同");
    }
    const std::string role = to_lower(role_);
    if (role != "sdk" && role != "remote") {
        throw std::invalid_argument("role 只能是 sdk 或 remote");
    }
    if (!in_range<double>(control_rate_hz_, 5.0, 100.0)) {
        throw std::invalid_argument("control_rate_hz 必须在 5~100");
    }
    if (!in_range<double>(status_rate_hz_, 0.5, 20.0)) {
        throw std::invalid_argument("status_rate_hz 必须在 0.5~20");
    }
    if (!in_range<double>(control_request_period_sec_, 0.2, 10.0)) {
        throw std::invalid_argument("control_request_period_sec 必须在 0.2~10");
    }
    if (!in_range<double>(max_speed_scale_, 0.05, 0.6)) {
        throw std::invalid_argument("max_speed_scale 必须在 0.05~0.6");
    }
    if (!in_range<int64_t>(bridge_port_, kMinPort, kMaxPort)) {
        throw std::invalid_argument("bridge_port 无效");
    }
    if (!in_range<int64_t>(motion_timeout_ms_, 1000, 60000)) {
        throw std::invalid_argument("motion_timeout_ms 必须在 1000~60000");
    }
    if (!in_range<int64_t>(lease_ms_, 300, 5000)) {
        throw std::invalid_argument("lease_ms 必须在 300~5000");
    }
    if (!in_range<int64_t>(api_link_timeout_ms_, 300, 5000)) {
        throw std::invalid_argument("api_link_timeout_ms 必须在 300~5000");
    }
    if (position_tolerance_m_ <= 0.0 || angle_tolerance_deg_ <= 0.0 ||
        acceptance_m_ <= 0.0 || acceptance_deg_ <= 0.0)
    {
        throw std::invalid_argument("容差必须为正数");
    }
    if (!in_range<int64_t>(no_progress_timeout_ms_, 500, 10000)) {
        throw std::invalid_argument("no_progress_timeout_ms 必须在 500~10000");
    }
    if (!in_range<int64_t>(settle_ms_, 100, 3000)) {
        throw std::invalid_argument("settle_ms 必须在 100~3000");
    }
    for (const double sign : {sdk_forward_sign_, sdk_strafe_sign_, sdk_yaw_sign_,
                              yaw_feedback_sign_})
    {
        if (std::abs(sign) != 1.0) {
            throw std::invalid_argument("轴符号参数只能是 1 或 -1");
        }
    }
    if (!is_posture_code(static_cast<int>(estop_recovery_cmd_))) {
        throw std::invalid_argument("estop_recovery_cmd 必须是姿态命令码");
    }
    if (!in_range<double>(warn_period_sec_, 0.5, 60.0)) {
        throw std::invalid_argument("warn_period_sec 必须在 0.5~60");
    }
    if (!in_range<double>(max_forward_m_, 0.1, 5.0) ||
        !in_range<double>(max_strafe_m_, 0.1, 3.0) ||
        !in_range<double>(max_turn_deg_, 1.0, 180.0))
    {
        throw std::invalid_argument("max_forward_m/max_strafe_m/max_turn_deg 越界");
    }
}

bool AgentControlNode::role_is_sdk() const
{
    return to_lower(role_) == "sdk";
}

zsibot::Role AgentControlNode::parse_role() const
{
    return role_is_sdk() ? zsibot::Role::ROLE_SDK : zsibot::Role::ROLE_REMOTE;
}

bool AgentControlNode::is_posture_code(int code) const
{
    return code == kCmdStandUp || code == kCmdSitDown || code == kCmdMoveMode ||
           code == kCmdBalanceStand || code == kCmdLock;
}

bool AgentControlNode::is_allowed_command(int code) const
{
    return is_posture_code(code);
}

int AgentControlNode::posture_expected_mode(int code) const
{
    switch (code) {
        case kCmdStandUp: return static_cast<int>(zsibot::ControlMode::CM_STAND_UP);
        case kCmdSitDown: return static_cast<int>(zsibot::ControlMode::CM_SIT_DOWN);
        case kCmdBalanceStand:
            return static_cast<int>(zsibot::ControlMode::CM_BALANCE_STAND_MODE);
        case kCmdLock: return static_cast<int>(zsibot::ControlMode::CM_LOCK_MODE);
        case kCmdMoveMode: return 0;  // no reliable feedback string; accept after hold
        default: return 0;
    }
}

std::string AgentControlNode::posture_name(int code) const
{
    switch (code) {
        case kCmdStandUp: return "stand_up";
        case kCmdSitDown: return "sit_down";
        case kCmdMoveMode: return "move_mode";
        case kCmdBalanceStand: return "balance_stand";
        case kCmdLock: return "lock";
        default: return "unknown";
    }
}

bool AgentControlNode::api_link_alive(TimePoint /*now*/) const
{
    if (!link_ok_.load()) {
        return false;
    }
    const int64_t last = last_client_rx_ms_.load();
    if (last <= 0) {
        return false;
    }
    return now_ms_epoch() - last <= api_link_timeout_ms_;
}

bool AgentControlNode::motion_ready(std::string & reason) const
{
    if (!connected_) {
        reason = "not_connected";
        return false;
    }
    if (!in_control_) {
        reason = "no_control";
        return false;
    }
    if (estop_latched_) {
        reason = "estop_latched";
        return false;
    }
    if (!odom_ready_) {
        reason = "odometry_not_ready";
        return false;
    }
    const int expected_function = role_is_sdk()
        ? static_cast<int>(zsibot::FunctionMode::FM_SDK)
        : static_cast<int>(zsibot::FunctionMode::FM_REMOTE);
    if (function_mode_ != expected_function) {
        reason = "wrong_function_mode";
        return false;
    }
    if (control_mode_ != static_cast<int>(zsibot::ControlMode::CM_STAND_UP) &&
        control_mode_ != static_cast<int>(zsibot::ControlMode::CM_MOVE_MODE) &&
        control_mode_ != static_cast<int>(zsibot::ControlMode::CM_BALANCE_STAND_MODE))
    {
        reason = "not_standing";
        return false;
    }
    if (posture_active_) {
        reason = "posture_active";
        return false;
    }
    if (!api_link_alive(std::chrono::steady_clock::now())) {
        reason = "api_link_lost";
        return false;
    }
    reason.clear();
    return true;
}

void AgentControlNode::enqueue_outbound(std::string line)
{
    std::lock_guard<std::mutex> lock(outbound_mutex_);
    if (outbound_.size() > 4096) {
        outbound_.pop_front();
    }
    outbound_.push_back(std::move(line));
}

void AgentControlNode::send_ack(
    const std::string & request_id, bool accepted, const std::string & code,
    const std::string & message, bool duplicate)
{
    std::ostringstream out;
    out << "A:{\"request_id\":\"" << json_escape(request_id) << "\""
        << ",\"accepted\":" << json_bool(accepted)
        << ",\"duplicate\":" << json_bool(duplicate)
        << ",\"code\":\"" << json_escape(code) << "\""
        << ",\"message\":\"" << json_escape(message) << "\"}";
    enqueue_outbound(out.str());
}

void AgentControlNode::send_result(
    const std::string & request_id, const std::string & kind,
    const MotionResult & result)
{
    std::ostringstream out;
    out << "R:{\"request_id\":\"" << json_escape(request_id) << "\""
        << ",\"kind\":\"" << json_escape(kind) << "\""
        << ",\"state\":\"" << json_escape(result.state) << "\""
        << ",\"result\":{\"forward_m\":" << json_number(result.forward_m)
        << ",\"strafe_m\":" << json_number(result.strafe_m)
        << ",\"yaw_deg\":" << json_number(result.yaw_deg)
        << ",\"elapsed_ms\":" << json_number(result.elapsed_s * 1000.0, 0) << "}"
        << ",\"error\":";
    if (result.code.empty() && result.message.empty()) {
        out << "null";
    } else {
        out << "{\"code\":\"" << json_escape(result.code) << "\",\"message\":\""
            << json_escape(result.message) << "\"}";
    }
    out << ",\"elapsed_ms\":" << json_number(result.elapsed_s * 1000.0, 0) << "}";
    enqueue_outbound(out.str());
}

void AgentControlNode::send_protocol_error(const std::string & code, const std::string & message)
{
    std::ostringstream out;
    out << "E:{\"code\":\"" << json_escape(code) << "\",\"message\":\""
        << json_escape(message) << "\"}";
    enqueue_outbound(out.str());
}

void AgentControlNode::bridge_server_loop()
{
    listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        RCLCPP_ERROR(get_logger(), "无法创建 TCP 控制桥 socket");
        return;
    }
    int reuse = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(static_cast<uint16_t>(bridge_port_));
    if (bind(listen_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0 ||
        listen(listen_fd_, 1) < 0)
    {
        RCLCPP_ERROR(
            get_logger(), "TCP 控制桥无法监听 127.0.0.1:%lld（端口被占用？）",
            static_cast<long long>(bridge_port_));
        close(listen_fd_);
        listen_fd_ = -1;
        return;
    }
    RCLCPP_INFO(
        get_logger(), "TCP 控制桥监听 127.0.0.1:%lld",
        static_cast<long long>(bridge_port_));

    std::string send_buffer;
    auto close_client = [this, &send_buffer]() {
        std::lock_guard<std::mutex> lock(client_mutex_);
        if (client_fd_ >= 0) {
            close(client_fd_);
            client_fd_ = -1;
        }
        link_ok_ = false;
        last_client_rx_ms_ = 0;
        read_buffer_.clear();
        send_buffer.clear();
    };

    while (!bridge_stop_.load()) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(listen_fd_, &read_fds);
        int max_fd = listen_fd_;
        int client_fd = -1;
        {
            std::lock_guard<std::mutex> lock(client_mutex_);
            client_fd = client_fd_;
        }
        if (client_fd >= 0) {
            FD_SET(client_fd, &read_fds);
            max_fd = std::max(max_fd, client_fd);
        }

        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;
        const int ready = select(max_fd + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ready < 0 && errno != EINTR) {
            break;
        }

        if (ready > 0 && FD_ISSET(listen_fd_, &read_fds)) {
            const int new_fd = accept(listen_fd_, nullptr, nullptr);
            if (new_fd >= 0) {
                close_client();
                {
                    std::lock_guard<std::mutex> lock(client_mutex_);
                    client_fd_ = new_fd;
                    read_buffer_.clear();
                }
                send_buffer.clear();
                link_ok_ = true;
                last_client_rx_ms_ = now_ms_epoch();
                RCLCPP_INFO(get_logger(), "API 控制桥已连接");
            }
        }

        client_fd = -1;
        {
            std::lock_guard<std::mutex> lock(client_mutex_);
            client_fd = client_fd_;
        }
        if (ready > 0 && client_fd >= 0 && FD_ISSET(client_fd, &read_fds)) {
            char buffer[1024];
            const ssize_t received = recv(client_fd, buffer, sizeof(buffer), 0);
            if (received <= 0) {
                close_client();
                RCLCPP_INFO(get_logger(), "API 控制桥已断开");
            } else {
                {
                    std::lock_guard<std::mutex> lock(client_mutex_);
                    read_buffer_.append(buffer, static_cast<std::size_t>(received));
                    if (read_buffer_.size() > 8192) {
                        read_buffer_.clear();
                    }
                }
                std::string local_lines;
                {
                    std::lock_guard<std::mutex> lock(client_mutex_);
                    std::size_t newline = 0;
                    while ((newline = read_buffer_.find('\n')) != std::string::npos) {
                        local_lines.append(read_buffer_, 0, newline + 1);
                        read_buffer_.erase(0, newline + 1);
                    }
                }
                std::size_t offset = 0;
                while (offset < local_lines.size()) {
                    const std::size_t newline = local_lines.find('\n', offset);
                    const std::string line = local_lines.substr(
                        offset, newline == std::string::npos ? std::string::npos : newline - offset);
                    offset = newline == std::string::npos ? local_lines.size() : newline + 1;
                    bridge_handle_line(line);
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(outbound_mutex_);
            while (!outbound_.empty() && send_buffer.size() < 256 * 1024) {
                send_buffer += outbound_.front();
                send_buffer.push_back('\n');
                outbound_.pop_front();
            }
        }

        client_fd = -1;
        {
            std::lock_guard<std::mutex> lock(client_mutex_);
            client_fd = client_fd_;
        }
        if (client_fd >= 0 && !send_buffer.empty()) {
            const ssize_t sent = send(
                client_fd, send_buffer.data(), send_buffer.size(),
                MSG_NOSIGNAL | MSG_DONTWAIT);
            if (sent > 0) {
                send_buffer.erase(0, static_cast<std::size_t>(sent));
            } else if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                close_client();
                RCLCPP_WARN(get_logger(), "API 控制桥发送失败，已断开");
            }
            if (send_buffer.size() > 256 * 1024) {
                close_client();
                RCLCPP_WARN(get_logger(), "API 控制桥发送缓冲过大，已断开");
            }
        }
    }

    close_client();
    if (listen_fd_ >= 0) {
        close(listen_fd_);
        listen_fd_ = -1;
    }
    RCLCPP_INFO(get_logger(), "TCP 控制桥已停止");
}

void AgentControlNode::bridge_handle_line(const std::string & line)
{
    if (line.empty()) {
        return;
    }
    const ParsedCommand command = parse_command(line);
    if (command.kind == CommandKind::INVALID) {
        send_protocol_error("bad_line", command.error);
        return;
    }
    last_client_rx_ms_ = now_ms_epoch();
    api_rx_count_.fetch_add(1);
    {
        std::lock_guard<std::mutex> lock(inbox_mutex_);
        if (inbox_.size() >= 256) {
            inbox_.pop_front();
        }
        inbox_.push_back(command);
    }
}

void AgentControlNode::drain_inbox()
{
    std::deque<ParsedCommand> commands;
    {
        std::lock_guard<std::mutex> lock(inbox_mutex_);
        commands.swap(inbox_);
    }
    if (commands.empty()) {
        return;
    }
    std::stable_sort(
        commands.begin(), commands.end(),
        [](const ParsedCommand & left, const ParsedCommand & right) {
            auto priority = [](CommandKind kind) {
                switch (kind) {
                    case CommandKind::ESTOP: return 0;
                    case CommandKind::STOP: return 1;
                    case CommandKind::ECLEAR: return 2;
                    default: return 3;
                }
            };
            return priority(left.kind) < priority(right.kind);
        });
    const TimePoint now = std::chrono::steady_clock::now();
    for (const ParsedCommand & command : commands) {
        handle_command(command, now);
    }
}

void AgentControlNode::handle_command(const ParsedCommand & command, TimePoint now)
{
    switch (command.kind) {
        case CommandKind::PING: break;
        case CommandKind::MOVE: handle_move(command, now); break;
        case CommandKind::TURN: handle_turn(command, now); break;
        case CommandKind::CMD: handle_posture(command, now); break;
        case CommandKind::STOP: handle_stop(command, now); break;
        case CommandKind::ESTOP: handle_estop(command, now); break;
        case CommandKind::ECLEAR: handle_estop_clear(command, now); break;
        case CommandKind::KEEP: handle_keep(command, now); break;
        case CommandKind::INVALID: break;
    }
}

bool AgentControlNode::is_motion_result_cached(
    const std::string & request_id, MotionResult & result) const
{
    const auto found = result_cache_.find(request_id);
    if (found == result_cache_.end()) {
        return false;
    }
    result = found->second;
    return true;
}

void AgentControlNode::cache_motion_result(
    const std::string & request_id, const MotionResult & result)
{
    result_cache_[request_id] = result;
    result_order_.push_back(request_id);
    while (result_order_.size() > 64) {
        result_cache_.erase(result_order_.front());
        result_order_.pop_front();
    }
}

void AgentControlNode::handle_move(const ParsedCommand & command, TimePoint now)
{
    MotionResult cached;
    if (is_motion_result_cached(command.request_id, cached)) {
        send_result(command.request_id, "move", cached);
        send_ack(command.request_id, true, "", "", true);
        return;
    }
    if (motion_.active() || posture_active_) {
        send_ack(command.request_id, false, "busy", "another motion is active");
        return;
    }
    if (std::abs(command.forward_m) > max_forward_m_ + 1e-9 ||
        std::abs(command.strafe_m) > max_strafe_m_ + 1e-9)
    {
        send_ack(command.request_id, false, "out_of_range", "distance exceeds driver limit");
        return;
    }
    if (command.max_speed_scale <= 0.0 ||
        command.max_speed_scale > max_speed_scale_ + 1e-9)
    {
        send_ack(command.request_id, false, "invalid_scale", "max_speed_scale exceeds driver limit");
        return;
    }
    if (!in_range<int>(command.timeout_ms, 1000, static_cast<int>(motion_timeout_ms_)) ||
        !in_range<int>(command.lease_ms, 300, 5000))
    {
        send_ack(command.request_id, false, "invalid_timeout", "timeout/lease out of range");
        return;
    }
    std::string reason;
    if (!motion_ready(reason)) {
        send_ack(command.request_id, false, "not_ready", reason);
        return;
    }

    const MotionFeedback feedback{
        position_x_, position_y_, yaw_ * yaw_feedback_sign_,
        body_vx_, body_vy_, 0.0, true};
    MotionRequest request;
    request.request_id = command.request_id;
    request.turn = false;
    request.forward_m = command.forward_m;
    request.strafe_m = command.strafe_m;
    request.max_speed_scale = std::min(command.max_speed_scale, max_speed_scale_);
    request.timeout_ms = command.timeout_ms;
    request.lease_ms = command.lease_ms;
    request.lease_deadline = now + std::chrono::milliseconds(command.lease_ms);

    std::string error;
    if (!motion_.start(request, feedback, now, error)) {
        send_ack(command.request_id, false, "invalid_motion", error);
        return;
    }
    active_job_ = JobSnapshot{};
    active_job_.present = true;
    active_job_.active = true;
    active_job_.request_id = command.request_id;
    active_job_.kind = "move";
    active_job_.phase = "moving";
    active_job_.state = "running";
    active_job_.requested_forward_m = command.forward_m;
    active_job_.requested_strafe_m = command.strafe_m;
    active_job_.lease_remaining_ms = command.lease_ms;
    last_error_.clear();
    send_ack(command.request_id, true, "", "");
}

void AgentControlNode::handle_turn(const ParsedCommand & command, TimePoint now)
{
    MotionResult cached;
    if (is_motion_result_cached(command.request_id, cached)) {
        send_result(command.request_id, "turn", cached);
        send_ack(command.request_id, true, "", "", true);
        return;
    }
    if (motion_.active() || posture_active_) {
        send_ack(command.request_id, false, "busy", "another motion is active");
        return;
    }
    const double max_angle = max_turn_deg_ * kPi / 180.0;
    if (std::abs(command.angle_rad) > max_angle + 1e-9) {
        send_ack(command.request_id, false, "out_of_range", "angle exceeds driver limit");
        return;
    }
    if (command.max_speed_scale <= 0.0 ||
        command.max_speed_scale > max_speed_scale_ + 1e-9)
    {
        send_ack(command.request_id, false, "invalid_scale", "max_speed_scale exceeds driver limit");
        return;
    }
    if (!in_range<int>(command.timeout_ms, 1000, static_cast<int>(motion_timeout_ms_)) ||
        !in_range<int>(command.lease_ms, 300, 5000))
    {
        send_ack(command.request_id, false, "invalid_timeout", "timeout/lease out of range");
        return;
    }
    std::string reason;
    if (!motion_ready(reason)) {
        send_ack(command.request_id, false, "not_ready", reason);
        return;
    }

    const MotionFeedback feedback{
        position_x_, position_y_, yaw_ * yaw_feedback_sign_,
        body_vx_, body_vy_, 0.0, true};
    MotionRequest request;
    request.request_id = command.request_id;
    request.turn = true;
    request.angle_rad = command.angle_rad;
    request.max_speed_scale = std::min(command.max_speed_scale, max_speed_scale_);
    request.timeout_ms = command.timeout_ms;
    request.lease_ms = command.lease_ms;
    request.lease_deadline = now + std::chrono::milliseconds(command.lease_ms);

    std::string error;
    if (!motion_.start(request, feedback, now, error)) {
        send_ack(command.request_id, false, "invalid_motion", error);
        return;
    }
    active_job_ = JobSnapshot{};
    active_job_.present = true;
    active_job_.active = true;
    active_job_.request_id = command.request_id;
    active_job_.kind = "turn";
    active_job_.phase = "moving";
    active_job_.state = "running";
    active_job_.requested_angle_deg = command.angle_rad * 180.0 / kPi;
    active_job_.lease_remaining_ms = command.lease_ms;
    last_error_.clear();
    send_ack(command.request_id, true, "", "");
}

void AgentControlNode::handle_posture(const ParsedCommand & command, TimePoint now)
{
    if (!is_allowed_command(command.cmd_code)) {
        send_ack(command.request_id, false, "unsupported_command", "command not allowed");
        return;
    }
    if (motion_.active() || posture_active_) {
        send_ack(command.request_id, false, "busy", "motion active; stop first");
        return;
    }
    if (!connected_ || !in_control_ || estop_latched_ || !api_link_alive(now)) {
        send_ack(command.request_id, false, "not_ready", "robot not ready");
        return;
    }
    zsibot::ZsibotExecutor * executor = holder_->get();
    if (executor == nullptr) {
        send_ack(command.request_id, false, "not_ready", "executor not ready");
        return;
    }

    set_remote(*executor, make_sdk_axes(0.0f, 0.0f, 0.0f));
    send_cmd(*executor, command.cmd_code);
    posture_active_ = true;
    posture_request_id_ = command.request_id;
    posture_code_ = command.cmd_code;
    posture_expected_mode_ = posture_expected_mode(command.cmd_code);
    posture_start_ = now;
    posture_hold_until_ = now + kPostureHold;
    active_job_ = JobSnapshot{};
    active_job_.present = true;
    active_job_.active = true;
    active_job_.request_id = command.request_id;
    active_job_.kind = "posture";
    active_job_.phase = "holding";
    active_job_.state = "running";
    last_command_code_ = command.cmd_code;
    last_command_time_ = now;
    last_error_.clear();
    send_ack(command.request_id, true, "", "");
}

void AgentControlNode::handle_stop(const ParsedCommand & command, TimePoint now)
{
    const MotionFeedback feedback{
        position_x_, position_y_, yaw_ * yaw_feedback_sign_,
        body_vx_, body_vy_, 0.0, true};
    if (motion_.active()) {
        const MotionResult result = motion_.cancel(
            "interrupted", "stopped by client", feedback, now);
        finish_motion(result);
    }
    if (posture_active_) {
        MotionResult result;
        result.finished = true;
        result.state = "aborted";
        result.code = "interrupted";
        result.message = "posture command interrupted";
        result.elapsed_s = std::chrono::duration<double>(now - posture_start_).count();
        send_result(posture_request_id_, "posture", result);
        last_job_ = active_job_;
        last_job_.active = false;
        last_job_.state = result.state;
        last_job_.measured_yaw_deg = 0.0;
        last_job_.elapsed_ms = result.elapsed_s * 1000.0;
        last_job_.code = result.code;
        last_job_.message = result.message;
        have_last_job_ = true;
        posture_active_ = false;
        active_job_ = JobSnapshot{};
    }
    send_ack(command.request_id, true, "", "");
}

void AgentControlNode::handle_estop(const ParsedCommand & command, TimePoint now)
{
    estop_latched_ = true;
    estop_clear_pending_ = false;
    estop_recovery_sent_ = false;
    estop_started_ = now;
    estop_resend_count_ = 0;
    last_estop_send_ = TimePoint{};
    in_control_ = false;

    const MotionFeedback feedback{
        position_x_, position_y_, yaw_ * yaw_feedback_sign_,
        body_vx_, body_vy_, 0.0, true};
    if (motion_.active()) {
        const MotionResult result = motion_.cancel(
            "estop", "emergency stop", feedback, now);
        finish_motion(result);
    }
    if (posture_active_) {
        MotionResult result;
        result.finished = true;
        result.state = "aborted";
        result.code = "estop";
        result.message = "emergency stop";
        result.elapsed_s = std::chrono::duration<double>(now - posture_start_).count();
        send_result(posture_request_id_, "posture", result);
        last_job_ = active_job_;
        last_job_.active = false;
        last_job_.state = result.state;
        last_job_.elapsed_ms = result.elapsed_s * 1000.0;
        last_job_.code = result.code;
        last_job_.message = result.message;
        have_last_job_ = true;
        posture_active_ = false;
        active_job_ = JobSnapshot{};
    }

    zsibot::ZsibotExecutor * executor = holder_->get();
    if (executor != nullptr) {
        set_remote(*executor, make_sdk_axes(0.0f, 0.0f, 0.0f));
        send_cmd(*executor, kCmdEmergencyStop);
        last_estop_send_ = now;
        estop_resend_count_ = 1;
    }
    last_error_ = "emergency stop latched";
    send_ack(command.request_id, true, "", "");
}

void AgentControlNode::handle_estop_clear(const ParsedCommand & command, TimePoint now)
{
    if (!estop_latched_) {
        send_ack(command.request_id, false, "not_latched", "estop is not latched");
        return;
    }
    estop_clear_pending_ = true;
    estop_clear_start_ = now;
    estop_recovery_sent_ = false;
    send_ack(command.request_id, true, "", "");
}

void AgentControlNode::handle_keep(const ParsedCommand & command, TimePoint now)
{
    if (motion_.active() && motion_.request().request_id == command.request_id) {
        motion_.extend_lease(now + std::chrono::milliseconds(lease_ms_));
        active_job_.lease_remaining_ms = lease_ms_;
    }
}

bool AgentControlNode::ensure_executor(TimePoint now)
{
    if (holder_->ensure(
            parse_role(), robot_ip_, static_cast<uint32_t>(send_port_),
            static_cast<uint32_t>(recv_port_), now))
    {
        return true;
    }
    std::string error;
    if (holder_->take_failure(error)) {
        last_error_ = error;
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), static_cast<int64_t>(warn_period_sec_ * 1000.0),
            "创建 ZsibotExecutor 失败: %s", error.c_str());
    }
    return false;
}

void AgentControlNode::refresh_feedback(zsibot::ZsibotExecutor & executor)
{
    try {
        connected_ = executor.IsConnected();
        const auto battery = executor.GetBatteryInfo();
        battery_.power = battery.power;
        battery_.volt = battery.volt;
        battery_.current = battery.current;
        battery_.temp = battery.temp;
        battery_.error = battery.error;
        body_temp_c_ = executor.GetTemperature();
        speed_level_ = static_cast<int>(executor.GetSpeedLevel());
        function_mode_ = static_cast<int>(executor.GetFunctionMode());
        control_mode_ = static_cast<int>(executor.GetControlMode());
        motion_mode_ = static_cast<int>(executor.GetMotionMode());
        motion_type_ = static_cast<int>(executor.GetMotionType());
        model_ = model_name(static_cast<int>(executor.GetModel()));
        const auto rpy = executor.GetRPY();
        const auto position = executor.GetPosition();
        const auto world = executor.GetWorldVelocity();
        const auto body = executor.GetBodyVelocity();
        const auto faults = executor.GetFaultInfo();

        const bool finite =
            std::isfinite(rpy[0]) && std::isfinite(rpy[1]) && std::isfinite(rpy[2]) &&
            std::isfinite(position[0]) && std::isfinite(position[1]) &&
            std::isfinite(position[2]) && std::isfinite(world[0]) &&
            std::isfinite(world[1]) && std::isfinite(world[2]) &&
            std::isfinite(body[0]) && std::isfinite(body[1]) && std::isfinite(body[2]);

        if (finite) {
            roll_ = rpy[0];
            pitch_ = rpy[1];
            const double new_yaw = rpy[2];
            if (have_last_odom_) {
                double delta = new_yaw - last_yaw_;
                while (delta > kPi) delta -= 2.0 * kPi;
                while (delta < -kPi) delta += 2.0 * kPi;
                yaw_unwrapped_ += delta;
            } else {
                yaw_unwrapped_ = new_yaw;
                have_last_odom_ = true;
            }
            last_yaw_ = new_yaw;
            yaw_ = new_yaw;
            position_x_ = position[0];
            position_y_ = position[1];
            position_z_ = position[2];
            world_vx_ = world[0];
            world_vy_ = world[1];
            world_vz_ = world[2];
            body_vx_ = body[0];
            body_vy_ = body[1];
            body_vz_ = body[2];
            if (odom_finite_since_ == TimePoint{}) {
                odom_finite_since_ = std::chrono::steady_clock::now();
            }
            odom_ready_ = std::chrono::steady_clock::now() - odom_finite_since_ >=
                std::chrono::seconds(1);
        } else {
            odom_ready_ = false;
            odom_finite_since_ = TimePoint{};
        }

        faults_.clear();
        for (const auto & fault : faults) {
            FaultSnapshot snapshot;
            snapshot.module = fault.module;
            snapshot.submodule = fault.submodule;
            snapshot.level = fault.level;
            snapshot.info = fault.info;
            snapshot.code = static_cast<int>(fault.error_code);
            faults_.push_back(snapshot);
        }

        if (connected_ && !estop_latched_ &&
            control_mode_ == static_cast<int>(zsibot::ControlMode::CM_EMERGENCY_STOP))
        {
            estop_latched_ = true;
            estop_clear_pending_ = false;
            estop_recovery_sent_ = false;
            estop_started_ = std::chrono::steady_clock::now();
            estop_resend_count_ = 0;
            last_estop_send_ = TimePoint{};
            in_control_ = false;
            last_error_ = "robot feedback reports emergency stop";
            RCLCPP_WARN(get_logger(), "机器人反馈处于急停状态，锁定运动输出");
        }
    } catch (const std::exception & exception) {
        connected_ = false;
        in_control_ = false;
        odom_ready_ = false;
        last_error_ = std::string("读取状态失败: ") + exception.what();
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), static_cast<int64_t>(warn_period_sec_ * 1000.0),
            "%s", last_error_.c_str());
    }
}

void AgentControlNode::update_control_state(
    zsibot::ZsibotExecutor & executor, TimePoint now)
{
    if (!connected_) {
        in_control_ = false;
        control_confirm_ticks_ = 0;
        return;
    }
    const bool emergency =
        control_mode_ == static_cast<int>(zsibot::ControlMode::CM_EMERGENCY_STOP);
    if (role_is_sdk()) {
        const bool ready =
            function_mode_ == static_cast<int>(zsibot::FunctionMode::FM_SDK) && !emergency;
        if (in_control_) {
            if (!ready) {
                in_control_ = false;
                control_confirm_ticks_ = 0;
                force_slow_sent_ = false;
                last_error_ = "SDK control lost";
                RCLCPP_WARN(get_logger(), "SDK 控制权丢失，停止运动");
            }
            return;
        }
        if (auto_request_control_ &&
            (last_control_request_ == TimePoint{} ||
             now - last_control_request_ >=
                 std::chrono::duration<double>(control_request_period_sec_)))
        {
            send_cmd(executor, kCmdSdkControlRight);
            last_control_request_ = now;
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(),
                static_cast<int64_t>(warn_period_sec_ * 1000.0),
                "正在请求 SDK 控制权...");
        }
        if (ready) {
            ++control_confirm_ticks_;
            if (control_confirm_ticks_ >= 2) {
                in_control_ = true;
                control_confirm_ticks_ = 0;
                force_slow_sent_ = false;
                last_error_.clear();
                RCLCPP_INFO(get_logger(), "已取得 SDK 控制权");
            }
        } else {
            control_confirm_ticks_ = 0;
        }
        return;
    }

    const bool ready =
        function_mode_ == static_cast<int>(zsibot::FunctionMode::FM_REMOTE) && !emergency;
    if (ready != in_control_) {
        in_control_ = ready;
        force_slow_sent_ = false;
        if (ready) {
            RCLCPP_INFO(get_logger(), "已取得遥控模式运动输出");
        } else {
            RCLCPP_WARN(get_logger(), "遥控模式不可用，运动指令可能被丢弃");
        }
    }
}

void AgentControlNode::handle_estop_tick(
    zsibot::ZsibotExecutor & executor, TimePoint now)
{
    in_control_ = false;
    set_remote(executor, make_sdk_axes(0.0f, 0.0f, 0.0f));

    // Never start recovery until all E-stop resends have been attempted. A
    // clear request can arrive before the 200 ms resend timer is due; falling
    // through early would send the stand-up command and then a late 90 would
    // re-latch the robot.
    if (estop_resend_count_ < kEstopResendCount) {
        if (estop_resend_count_ == 0 ||
            now - last_estop_send_ >= kEstopResendPeriod)
        {
            send_cmd(executor, kCmdEmergencyStop);
            last_estop_send_ = now;
            ++estop_resend_count_;
        }
        return;
    }

    if (!estop_clear_pending_) {
        return;
    }

    const bool control_ready =
        connected_ &&
        (role_is_sdk()
             ? function_mode_ == static_cast<int>(zsibot::FunctionMode::FM_SDK)
             : function_mode_ == static_cast<int>(zsibot::FunctionMode::FM_REMOTE));
    if (!control_ready) {
        if (role_is_sdk() && auto_request_control_ && connected_ &&
            (last_control_request_ == TimePoint{} ||
             now - last_control_request_ >=
                 std::chrono::duration<double>(control_request_period_sec_)))
        {
            send_cmd(executor, kCmdSdkControlRight);
            last_control_request_ = now;
        }
        if (now - estop_clear_start_ >= kRecoveryTimeout) {
            estop_clear_pending_ = false;
            estop_recovery_sent_ = false;
            last_error_ = "急停恢复超时：未能重新取得控制权";
            RCLCPP_ERROR(get_logger(), "%s", last_error_.c_str());
        }
        return;
    }

    if (!estop_recovery_sent_) {
        send_cmd(executor, static_cast<int>(estop_recovery_cmd_));
        estop_recovery_sent_ = true;
        estop_recovery_time_ = now;
        RCLCPP_INFO(
            get_logger(), "急停恢复：发送姿态命令 %s",
            posture_name(static_cast<int>(estop_recovery_cmd_)).c_str());
        return;
    }

    if (now - estop_recovery_time_ >= kRecoveryConfirmDelay &&
        control_mode_ != static_cast<int>(zsibot::ControlMode::CM_EMERGENCY_STOP))
    {
        estop_latched_ = false;
        estop_clear_pending_ = false;
        estop_recovery_sent_ = false;
        estop_resend_count_ = 0;
        in_control_ = true;
        force_slow_sent_ = false;
        last_error_.clear();
        RCLCPP_INFO(get_logger(), "急停已解除");
    } else if (now - estop_clear_start_ >= kRecoveryTimeout) {
        estop_clear_pending_ = false;
        estop_recovery_sent_ = false;
        last_error_ = "急停恢复超时：机器人未离开急停状态";
        RCLCPP_ERROR(get_logger(), "%s", last_error_.c_str());
    }
}

void AgentControlNode::handle_posture_tick(
    zsibot::ZsibotExecutor & /*executor*/, TimePoint now)
{
    if (!posture_active_) {
        return;
    }
    if (now < posture_hold_until_) {
        return;
    }
    const bool confirmed = posture_expected_mode_ != 0 && control_mode_ == posture_expected_mode_;
    const bool accepted_without_feedback = posture_expected_mode_ == 0 &&
        now - posture_start_ >= std::chrono::milliseconds(500);
    if (confirmed || accepted_without_feedback) {
        MotionResult result;
        result.finished = true;
        result.success = true;
        result.state = "succeeded";
        result.elapsed_s = std::chrono::duration<double>(now - posture_start_).count();
        send_result(posture_request_id_, "posture", result);
        last_job_ = active_job_;
        last_job_.active = false;
        last_job_.state = "succeeded";
        last_job_.elapsed_ms = result.elapsed_s * 1000.0;
        have_last_job_ = true;
        posture_active_ = false;
        active_job_ = JobSnapshot{};
        return;
    }
    if (now - posture_start_ >= kPostureTimeout) {
        MotionResult result;
        result.finished = true;
        result.success = false;
        result.state = "failed";
        result.code = "posture_timeout";
        result.message = "posture command was not confirmed";
        result.elapsed_s = std::chrono::duration<double>(now - posture_start_).count();
        send_result(posture_request_id_, "posture", result);
        last_job_ = active_job_;
        last_job_.active = false;
        last_job_.state = "failed";
        last_job_.elapsed_ms = result.elapsed_s * 1000.0;
        last_job_.code = result.code;
        last_job_.message = result.message;
        have_last_job_ = true;
        posture_active_ = false;
        active_job_ = JobSnapshot{};
    }
}

void AgentControlNode::finish_motion(const MotionResult & result)
{
    if (!result.finished) {
        return;
    }
    const std::string request_id =
        active_job_.present ? active_job_.request_id : motion_.request().request_id;
    const std::string kind =
        active_job_.present ? active_job_.kind : (motion_.request().turn ? "turn" : "move");
    send_result(request_id, kind, result);
    cache_motion_result(request_id, result);

    last_job_ = active_job_;
    last_job_.present = true;
    last_job_.active = false;
    last_job_.state = result.state;
    last_job_.measured_forward_m = result.forward_m;
    last_job_.measured_strafe_m = result.strafe_m;
    last_job_.measured_yaw_deg = result.yaw_deg;
    last_job_.elapsed_ms = result.elapsed_s * 1000.0;
    last_job_.lease_remaining_ms = 0;
    last_job_.code = result.code;
    last_job_.message = result.message;
    have_last_job_ = true;
    active_job_ = JobSnapshot{};
    motion_.reset();
}

void AgentControlNode::set_remote(
    zsibot::ZsibotExecutor & executor, const std::array<float, 4> & axes)
{
    try {
        executor.SetRemote(axes, std::array<float, 14>{});
        ++tx_count_;
    } catch (const std::exception & exception) {
        last_error_ = std::string("SetRemote 失败: ") + exception.what();
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), static_cast<int64_t>(warn_period_sec_ * 1000.0),
            "%s", last_error_.c_str());
    }
}

void AgentControlNode::send_cmd(zsibot::ZsibotExecutor & executor, int code)
{
    try {
        executor.SetCmd(static_cast<zsibot::CmdCode>(code));
    } catch (const std::exception & exception) {
        last_error_ = std::string("SetCmd 失败: ") + exception.what();
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), static_cast<int64_t>(warn_period_sec_ * 1000.0),
            "%s", last_error_.c_str());
    }
}

void AgentControlNode::update_rates(TimePoint now)
{
    if (rate_window_start_ == TimePoint{}) {
        rate_window_start_ = now;
        tx_count_ = 0;
        api_rx_count_ = 0;
        return;
    }
    const double elapsed = std::chrono::duration<double>(now - rate_window_start_).count();
    if (elapsed >= 1.0) {
        tx_hz_ = static_cast<double>(tx_count_) / elapsed;
        api_rx_hz_ = static_cast<double>(api_rx_count_.exchange(0)) / elapsed;
        tx_count_ = 0;
        rate_window_start_ = now;
    }
}

void AgentControlNode::publish_status(TimePoint now, bool force)
{
    if (!force && last_status_publish_ != TimePoint{} &&
        now - last_status_publish_ <
            std::chrono::duration<double>(1.0 / status_rate_hz_))
    {
        return;
    }
    last_status_publish_ = now;

    StatusSnapshot snapshot;
    snapshot.seq = ++status_seq_;
    snapshot.link_ok = link_ok_.load();
    snapshot.executor_ready = holder_->get() != nullptr;
    snapshot.connected = connected_;
    snapshot.in_control = in_control_;
    snapshot.estop_latched = estop_latched_;
    snapshot.estop_clear_pending = estop_clear_pending_;
    std::string reason;
    snapshot.motion_ready = motion_ready(reason);
    snapshot.motion_ready_reason = reason;
    snapshot.odom_ready = odom_ready_;
    snapshot.role = role_;
    snapshot.model = model_;
    snapshot.function_mode = function_mode_;
    snapshot.function_mode_name = function_mode_name(function_mode_);
    snapshot.control_mode = control_mode_;
    snapshot.control_mode_name = control_mode_name(control_mode_);
    snapshot.motion_mode = motion_mode_;
    snapshot.motion_mode_name = motion_mode_name(motion_mode_);
    snapshot.motion_type = motion_type_;
    snapshot.motion_type_name = motion_type_name(motion_type_);
    snapshot.battery = battery_;
    snapshot.body_temp_c = body_temp_c_;
    snapshot.speed_level = speed_level_;
    snapshot.speed_level_name = speed_level_name(speed_level_);
    snapshot.position_x = position_x_;
    snapshot.position_y = position_y_;
    snapshot.position_z = position_z_;
    snapshot.roll = roll_;
    snapshot.pitch = pitch_;
    snapshot.yaw = yaw_;
    snapshot.yaw_unwrapped = yaw_unwrapped_;
    snapshot.body_vx = body_vx_;
    snapshot.body_vy = body_vy_;
    snapshot.body_vz = body_vz_;
    snapshot.world_vx = world_vx_;
    snapshot.world_vy = world_vy_;
    snapshot.world_vz = world_vz_;
    snapshot.faults = faults_;
    snapshot.last_error = last_error_;
    snapshot.tx_hz = tx_hz_;
    snapshot.api_rx_hz = api_rx_hz_;

    if (motion_.active()) {
        snapshot.active_job = JobSnapshot{};
        snapshot.active_job.present = true;
        snapshot.active_job.active = true;
        snapshot.active_job.request_id = motion_.request().request_id;
        snapshot.active_job.kind = motion_.request().turn ? "turn" : "move";
        snapshot.active_job.phase = motion_.settling() ? "settling" : "moving";
        snapshot.active_job.state = "running";
        snapshot.active_job.requested_forward_m = motion_.request().forward_m;
        snapshot.active_job.requested_strafe_m = motion_.request().strafe_m;
        snapshot.active_job.requested_angle_deg =
            motion_.request().angle_rad * 180.0 / kPi;
        snapshot.active_job.measured_forward_m = motion_.progress_forward();
        snapshot.active_job.measured_strafe_m = motion_.progress_strafe();
        snapshot.active_job.measured_yaw_deg = motion_.progress_yaw_deg();
        snapshot.active_job.elapsed_ms = motion_.elapsed_seconds(now) * 1000.0;
        snapshot.active_job.lease_remaining_ms = motion_.lease_remaining_ms(now);
    } else if (posture_active_) {
        snapshot.active_job = active_job_;
        snapshot.active_job.active = true;
        snapshot.active_job.lease_remaining_ms = 0;
    }
    if (have_last_job_) {
        snapshot.last_job = last_job_;
    }

    std_msgs::msg::String message;
    message.data = build_status_json(snapshot);
    status_pub_->publish(message);
    enqueue_outbound("S:" + message.data);
}

void AgentControlNode::shutdown_motion()
{
    zsibot::ZsibotExecutor * executor = holder_->get();
    if (executor == nullptr) {
        return;
    }
    try {
        for (int count = 0; count < kShutdownZeroCount; ++count) {
            executor->SetRemote(
                make_sdk_axes(0.0f, 0.0f, 0.0f), std::array<float, 14>{});
            std::this_thread::sleep_for(kShutdownZeroPeriod);
        }
    } catch (const std::exception & exception) {
        RCLCPP_WARN(get_logger(), "退出时发送归零指令失败: %s", exception.what());
    }
}

void AgentControlNode::on_control_tick()
{
    const TimePoint now = std::chrono::steady_clock::now();
    update_rates(now);

    if (!ensure_executor(now)) {
        publish_status(now);
        return;
    }
    zsibot::ZsibotExecutor * executor = holder_->get();
    if (executor == nullptr) {
        publish_status(now);
        return;
    }

    refresh_feedback(*executor);
    drain_inbox();

    if (estop_latched_) {
        handle_estop_tick(*executor, now);
        publish_status(now);
        return;
    }

    update_control_state(*executor, now);

    if (force_slow_speed_ && in_control_ && connected_ && !force_slow_sent_) {
        send_cmd(*executor, kCmdSlow);
        force_slow_sent_ = true;
    }

    // If the API died or the control link disappeared, stop immediately.
    if (motion_.active() && !api_link_alive(now)) {
        const MotionFeedback feedback{
            position_x_, position_y_, yaw_ * yaw_feedback_sign_,
            body_vx_, body_vy_, 0.0, true};
        finish_motion(motion_.cancel(
            "api_link_lost", "API link lost", feedback, now));
    }
    if (motion_.active() && (!connected_ || !in_control_)) {
        const MotionFeedback feedback{
            position_x_, position_y_, yaw_ * yaw_feedback_sign_,
            body_vx_, body_vy_, 0.0, true};
        finish_motion(motion_.cancel(
            "control_lost", "control lost", feedback, now));
    }

    handle_posture_tick(*executor, now);

    std::array<float, 4> axes = make_sdk_axes(0.0f, 0.0f, 0.0f);
    if (motion_.active()) {
        const MotionFeedback feedback{
            position_x_, position_y_, yaw_ * yaw_feedback_sign_,
            body_vx_, body_vy_, 0.0, true};
        const MotionCommand command = motion_.step(feedback, now);
        if (motion_.result(feedback, now).finished) {
            finish_motion(motion_.result(feedback, now));
        } else {
            axes = make_sdk_axes(
                static_cast<float>(command.forward * sdk_forward_sign_),
                static_cast<float>(command.strafe * sdk_strafe_sign_),
                static_cast<float>(command.yaw * sdk_yaw_sign_));
        }
    }

    for (float & value : axes) {
        value = std::clamp(value, -1.0f, 1.0f);
    }
    set_remote(*executor, axes);
    publish_status(now);
}

}  // namespace agent_control_driver

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<agent_control_driver::AgentControlNode>();
        rclcpp::spin(node);
    } catch (const std::exception & exception) {
        RCLCPP_FATAL(
            rclcpp::get_logger("agent_control_driver"), "节点启动失败: %s",
            exception.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
