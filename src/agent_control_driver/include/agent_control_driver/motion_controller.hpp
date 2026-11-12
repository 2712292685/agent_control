#ifndef AGENT_CONTROL_DRIVER__MOTION_CONTROLLER_HPP_
#define AGENT_CONTROL_DRIVER__MOTION_CONTROLLER_HPP_

#include <chrono>
#include <string>

namespace agent_control_driver
{

using TimePoint = std::chrono::steady_clock::time_point;

struct MotionFeedback
{
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
    bool finite{true};
};

struct MotionRequest
{
    std::string request_id;
    bool turn{false};
    double forward_m{0.0};
    double strafe_m{0.0};
    double angle_rad{0.0};
    double max_speed_scale{0.35};
    int timeout_ms{35000};
    int lease_ms{1500};
    TimePoint lease_deadline{};
};

struct MotionCommand
{
    double forward{0.0};
    double strafe{0.0};
    double yaw{0.0};
};

struct MotionResult
{
    bool finished{false};
    bool success{false};
    std::string state;  // succeeded / failed / aborted / timed_out
    std::string code;
    std::string message;
    double forward_m{0.0};
    double strafe_m{0.0};
    double yaw_deg{0.0};
    double elapsed_s{0.0};
};

/// Pure closed-loop motion math. No SDK, ROS, sockets or clocks are read here;
/// every function receives the current time and feedback from the caller so it
/// can be unit-tested with a synthetic plant.
class MotionController
{
   public:
    bool start(
        const MotionRequest & request, const MotionFeedback & feedback,
        TimePoint now, std::string & error);
    MotionCommand step(const MotionFeedback & feedback, TimePoint now);
    MotionResult cancel(
        const std::string & code, const std::string & message,
        const MotionFeedback & feedback, TimePoint now);
    void reset();

    bool active() const;
    bool settling() const;
    const MotionRequest & request() const;
    MotionResult result(const MotionFeedback & feedback, TimePoint now) const;
    double progress_forward() const;
    double progress_strafe() const;
    double progress_yaw_deg() const;
    double elapsed_seconds(TimePoint now) const;
    long lease_remaining_ms(TimePoint now) const;
    void extend_lease(TimePoint deadline);

   private:
    enum class Phase
    {
        IDLE,
        RUNNING,
        SETTLING,
        DONE,
    };

    MotionCommand finish(
        bool success, const std::string & state, const std::string & code,
        const std::string & message);
    void update_progress(const MotionFeedback & feedback);
    bool abort_if_unhealthy(const MotionFeedback & feedback, TimePoint now);
    MotionCommand abort_with(
        const std::string & code, const std::string & message,
        const MotionFeedback & feedback, TimePoint now);

    Phase phase_{Phase::IDLE};
    MotionRequest request_{};
    TimePoint started_{};
    TimePoint settle_started_{};
    TimePoint last_progress_{};
    TimePoint last_position_stamp_{};
    TimePoint last_direction_check_{};
    double x0_{0.0};
    double y0_{0.0};
    double yaw0_{0.0};
    double last_yaw_{0.0};
    double yaw_accum_{0.0};
    double last_x_{0.0};
    double last_y_{0.0};
    double last_progress_metric_{0.0};
    double direction_sign_{1.0};
    bool correction_mode_{false};
    bool correction_attempted_{false};
    TimePoint correction_started_{};
    double progress_forward_{0.0};
    double progress_strafe_{0.0};
    MotionResult result_{};
};

}  // namespace agent_control_driver

#endif  // AGENT_CONTROL_DRIVER__MOTION_CONTROLLER_HPP_
