#ifndef AGENT_CONTROL_DRIVER__CONTROL_NODE_HPP_
#define AGENT_CONTROL_DRIVER__CONTROL_NODE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "agent_control_driver/executor_holder.hpp"
#include "agent_control_driver/motion_controller.hpp"
#include "agent_control_driver/protocol.hpp"
#include "agent_control_driver/status_builder.hpp"

namespace agent_control_driver
{

class AgentControlNode : public rclcpp::Node
{
   public:
    AgentControlNode();
    ~AgentControlNode() override;

    AgentControlNode(const AgentControlNode &) = delete;
    AgentControlNode & operator=(const AgentControlNode &) = delete;

   private:
    using TimePoint = std::chrono::steady_clock::time_point;

    // parameters
    std::string robot_ip_;
    int64_t send_port_{8081};
    int64_t recv_port_{8080};
    std::string role_;
    double control_rate_hz_{30.0};
    double status_rate_hz_{5.0};
    bool auto_request_control_{true};
    double control_request_period_sec_{1.0};
    double max_speed_scale_{0.35};
    int64_t bridge_port_{8766};
    int64_t motion_timeout_ms_{35000};
    int64_t lease_ms_{1500};
    int64_t api_link_timeout_ms_{1000};
    double position_tolerance_m_{0.03};
    double angle_tolerance_deg_{2.0};
    double acceptance_m_{0.05};
    double acceptance_deg_{5.0};
    int64_t no_progress_timeout_ms_{2000};
    int64_t settle_ms_{600};
    bool force_slow_speed_{true};
    double sdk_forward_sign_{1.0};
    double sdk_strafe_sign_{1.0};
    double sdk_yaw_sign_{1.0};
    double yaw_feedback_sign_{1.0};
    int64_t estop_recovery_cmd_{122};
    double warn_period_sec_{5.0};
    double max_forward_m_{5.0};
    double max_strafe_m_{3.0};
    double max_turn_deg_{180.0};

    // executor / feedback
    std::shared_ptr<ExecutorHolder> holder_;
    bool connected_{false};
    bool in_control_{false};
    int function_mode_{0};
    int control_mode_{0};
    int motion_mode_{0};
    int motion_type_{0};
    int speed_level_{0};
    std::string model_;
    std::string last_error_;
    std_msgs::msg::String status_message_;
    BatterySnapshot battery_;
    double body_temp_c_{0.0};
    std::vector<FaultSnapshot> faults_;
    std::vector<std::string> fault_lines_;

    // odometry
    double position_x_{0.0};
    double position_y_{0.0};
    double position_z_{0.0};
    double roll_{0.0};
    double pitch_{0.0};
    double yaw_{0.0};
    double yaw_unwrapped_{0.0};
    double last_yaw_{0.0};
    double body_vx_{0.0};
    double body_vy_{0.0};
    double body_vz_{0.0};
    double world_vx_{0.0};
    double world_vy_{0.0};
    double world_vz_{0.0};
    TimePoint odom_finite_since_{};
    bool odom_ready_{false};
    bool have_last_odom_{false};
    double last_odom_x_{0.0};
    double last_odom_y_{0.0};

    // motion / jobs
    MotionController motion_;
    JobSnapshot active_job_;
    JobSnapshot last_job_;
    std::unordered_map<std::string, MotionResult> result_cache_;
    std::deque<std::string> result_order_;
    bool have_last_job_{false};

    // posture command
    bool posture_active_{false};
    std::string posture_request_id_;
    int posture_code_{0};
    int posture_expected_mode_{0};
    TimePoint posture_start_{};
    TimePoint posture_hold_until_{};

    // estop
    bool estop_latched_{false};
    bool estop_clear_pending_{false};
    bool estop_recovery_sent_{false};
    TimePoint estop_started_{};
    TimePoint last_estop_send_{};
    TimePoint estop_clear_start_{};
    TimePoint estop_recovery_time_{};
    int estop_resend_count_{0};
    bool force_slow_sent_{false};

    // control state
    TimePoint last_control_request_{};
    int control_confirm_ticks_{0};
    TimePoint last_command_time_{};
    int last_command_code_{-1};

    // bridge
    std::thread bridge_thread_;
    std::atomic<bool> bridge_stop_{false};
    int listen_fd_{-1};
    int client_fd_{-1};
    std::mutex client_mutex_;
    std::string read_buffer_;
    std::deque<std::string> outbound_;
    std::mutex outbound_mutex_;
    std::deque<ParsedCommand> inbox_;
    std::mutex inbox_mutex_;
    std::atomic<bool> link_ok_{false};
    std::atomic<int64_t> last_client_rx_ms_{0};
    std::atomic<int64_t> pending_keep_request_{0};

    // rates
    TimePoint rate_window_start_{};
    uint64_t tx_count_{0};
    std::atomic<uint64_t> api_rx_count_{0};
    double tx_hz_{0.0};
    double api_rx_hz_{0.0};

    // timers / status
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    TimePoint last_status_publish_{};
    uint64_t status_seq_{0};

    void validate_parameters();
    void bridge_server_loop();
    void bridge_handle_line(const std::string & line);
    void enqueue_outbound(std::string line);
    void send_ack(
        const std::string & request_id, bool accepted, const std::string & code,
        const std::string & message, bool duplicate = false);
    void send_result(
        const std::string & request_id, const std::string & kind,
        const MotionResult & result);
    void send_protocol_error(const std::string & code, const std::string & message);

    void on_control_tick();
    void drain_inbox();
    void handle_command(const ParsedCommand & command, TimePoint now);
    void handle_move(const ParsedCommand & command, TimePoint now);
    void handle_turn(const ParsedCommand & command, TimePoint now);
    void handle_posture(const ParsedCommand & command, TimePoint now);
    void handle_stop(const ParsedCommand & command, TimePoint now);
    void handle_estop(const ParsedCommand & command, TimePoint now);
    void handle_estop_clear(const ParsedCommand & command, TimePoint now);
    void handle_keep(const ParsedCommand & command, TimePoint now);

    bool ensure_executor(TimePoint now);
    void refresh_feedback(zsibot::ZsibotExecutor & executor);
    void update_odometry();
    void update_control_state(zsibot::ZsibotExecutor & executor, TimePoint now);
    void handle_estop_tick(zsibot::ZsibotExecutor & executor, TimePoint now);
    void handle_posture_tick(zsibot::ZsibotExecutor & executor, TimePoint now);
    void finish_motion(const MotionResult & result);
    void set_remote(zsibot::ZsibotExecutor & executor, const std::array<float, 4> & axes);
    void send_cmd(zsibot::ZsibotExecutor & executor, int code);
    void publish_status(TimePoint now, bool force = false);
    void shutdown_motion();
    void update_rates(TimePoint now);

    bool role_is_sdk() const;
    zsibot::Role parse_role() const;
    bool is_posture_code(int code) const;
    bool is_allowed_command(int code) const;
    int posture_expected_mode(int code) const;
    std::string posture_name(int code) const;
    bool motion_ready(std::string & reason) const;
    bool api_link_alive(TimePoint now) const;
    bool is_motion_result_cached(const std::string & request_id, MotionResult & result) const;
    void cache_motion_result(const std::string & request_id, const MotionResult & result);
};

}  // namespace agent_control_driver

#endif  // AGENT_CONTROL_DRIVER__CONTROL_NODE_HPP_
