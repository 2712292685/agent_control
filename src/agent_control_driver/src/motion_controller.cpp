#include "agent_control_driver/motion_controller.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace agent_control_driver
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kPositionKp = 4.0;
constexpr double kAngleKp = 3.0;
constexpr double kMinCommand = 0.10;
constexpr double kMinCommandDistance = 0.06;
constexpr double kMinCommandAngleRad = 2.0 * kPi / 180.0;
constexpr double kPositionTolerance = 0.03;
constexpr double kAngleToleranceRad = 2.0 * kPi / 180.0;
constexpr double kAcceptancePosition = 0.05;
constexpr double kAcceptanceAngleRad = 5.0 * kPi / 180.0;
constexpr double kSettleSeconds = 0.6;
constexpr double kNoProgressWindow = 2.0;
constexpr double kDirectionWindow = 0.5;
constexpr double kDirectionTolerance = 0.02;
constexpr double kDirectionAngleTolerance = 0.5 * kPi / 180.0;
constexpr double kOdomJump = 0.5;
constexpr double kPredictiveSeconds = 0.12;
constexpr double kCorrectionWindowPosition = 0.10;
constexpr double kCorrectionWindowAngleRad = 10.0 * kPi / 180.0;
constexpr double kCorrectionMax = 0.12;
constexpr double kCorrectionMin = 0.06;
constexpr double kCorrectionTimeout = 1.5;

double wrap_pi(double angle)
{
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

double clamp(double value, double low, double high)
{
    return std::max(low, std::min(high, value));
}

double seconds_between(TimePoint from, TimePoint to)
{
    return std::chrono::duration<double>(to - from).count();
}

}  // namespace

bool MotionController::start(
    const MotionRequest & request, const MotionFeedback & feedback,
    TimePoint now, std::string & error)
{
    if (phase_ == Phase::RUNNING || phase_ == Phase::SETTLING) {
        error = "another motion is already active";
        return false;
    }
    if (!feedback.finite || !std::isfinite(feedback.x) || !std::isfinite(feedback.y) ||
        !std::isfinite(feedback.yaw))
    {
        error = "odometry feedback is not finite";
        return false;
    }
    if (!std::isfinite(request.forward_m) || !std::isfinite(request.strafe_m) ||
        !std::isfinite(request.angle_rad) || !std::isfinite(request.max_speed_scale) ||
        !std::isfinite(static_cast<double>(request.timeout_ms)) ||
        !std::isfinite(static_cast<double>(request.lease_ms)))
    {
        error = "motion request contains non-finite values";
        return false;
    }
    if (request.max_speed_scale <= 0.0 || request.timeout_ms <= 0 ||
        request.lease_ms <= 0)
    {
        error = "motion request limits are invalid";
        return false;
    }

    if (request.turn) {
        if (std::abs(request.angle_rad) < 1e-6) {
            error = "turn angle is zero";
            return false;
        }
        direction_sign_ = request.angle_rad >= 0.0 ? 1.0 : -1.0;
    } else {
        const double norm = std::hypot(request.forward_m, request.strafe_m);
        if (norm < 1e-6) {
            error = "move distance is zero";
            return false;
        }
        direction_sign_ = 1.0;
    }

    request_ = request;
    request_.lease_deadline = now + std::chrono::milliseconds(request.lease_ms);
    phase_ = Phase::RUNNING;
    started_ = now;
    settle_started_ = TimePoint{};
    last_progress_ = now;
    last_direction_check_ = now;
    last_position_stamp_ = now;
    x0_ = feedback.x;
    y0_ = feedback.y;
    yaw0_ = feedback.yaw;
    last_yaw_ = feedback.yaw;
    yaw_accum_ = 0.0;
    last_x_ = feedback.x;
    last_y_ = feedback.y;
    last_progress_metric_ = 0.0;
    progress_forward_ = 0.0;
    progress_strafe_ = 0.0;
    correction_mode_ = false;
    correction_attempted_ = false;
    correction_started_ = TimePoint{};
    result_ = MotionResult{};
    return true;
}

void MotionController::update_progress(const MotionFeedback & feedback)
{
    const double dx = feedback.x - x0_;
    const double dy = feedback.y - y0_;
    const double c = std::cos(yaw0_);
    const double s = std::sin(yaw0_);
    progress_forward_ = dx * c + dy * s;
    progress_strafe_ = -dx * s + dy * c;
    yaw_accum_ += wrap_pi(feedback.yaw - last_yaw_);
    last_yaw_ = feedback.yaw;
}

MotionCommand MotionController::abort_with(
    const std::string & code, const std::string & message,
    const MotionFeedback & feedback, TimePoint now)
{
    request_.lease_deadline = now;  // prevent further lease extensions
    update_progress(feedback);
    result_.finished = true;
    result_.success = false;
    result_.state = code == "timeout" ? "timed_out" : "aborted";
    result_.code = code;
    result_.message = message;
    result_.forward_m = progress_forward_;
    result_.strafe_m = progress_strafe_;
    result_.yaw_deg = yaw_accum_ * 180.0 / kPi;
    result_.elapsed_s = seconds_between(started_, now);
    phase_ = Phase::DONE;
    return MotionCommand{};
}

MotionCommand MotionController::finish(
    bool success, const std::string & state, const std::string & code,
    const std::string & message)
{
    result_.finished = true;
    result_.success = success;
    result_.state = state;
    result_.code = code;
    result_.message = message;
    result_.forward_m = progress_forward_;
    result_.strafe_m = progress_strafe_;
    result_.yaw_deg = yaw_accum_ * 180.0 / kPi;
    phase_ = Phase::DONE;
    return MotionCommand{};
}

MotionCommand MotionController::step(const MotionFeedback & feedback, TimePoint now)
{
    if (phase_ == Phase::IDLE || phase_ == Phase::DONE) {
        return MotionCommand{};
    }

    if (phase_ == Phase::SETTLING) {
        if (!feedback.finite || !std::isfinite(feedback.x) || !std::isfinite(feedback.y) ||
            !std::isfinite(feedback.yaw))
        {
            return abort_with("odom_invalid", "odometry feedback is not finite", feedback, now);
        }
        update_progress(feedback);
        if (seconds_between(settle_started_, now) >= kSettleSeconds) {
            const bool success = request_.turn
                ? std::abs(request_.angle_rad - yaw_accum_) <= kAcceptanceAngleRad
                : std::hypot(
                      request_.forward_m - progress_forward_,
                      request_.strafe_m - progress_strafe_) <= kAcceptancePosition;
            result_.elapsed_s = seconds_between(started_, now);
            const double final_error = request_.turn
                ? std::abs(request_.angle_rad - yaw_accum_)
                : std::hypot(
                      request_.forward_m - progress_forward_,
                      request_.strafe_m - progress_strafe_);
            const double strict_tolerance = request_.turn
                ? kAngleToleranceRad : kPositionTolerance;
            const bool strict = final_error <= strict_tolerance;
            if (success && (correction_attempted_ || strict)) {
                return finish(true, "succeeded", "", "");
            }
            const double correction_window = request_.turn
                ? kCorrectionWindowAngleRad : kCorrectionWindowPosition;
            if (!correction_attempted_ && final_error <= correction_window) {
                correction_attempted_ = true;
                correction_mode_ = true;
                correction_started_ = now;
                phase_ = Phase::RUNNING;
                return MotionCommand{};
            }
            if (success) {
                return finish(true, "succeeded", "", "");
            }
            return finish(false, "failed", "not_reached", "target not reached within acceptance window");
        }
        return MotionCommand{};
    }

    if (!feedback.finite || !std::isfinite(feedback.x) || !std::isfinite(feedback.y) ||
        !std::isfinite(feedback.yaw))
    {
        return abort_with("odom_invalid", "odometry feedback is not finite", feedback, now);
    }

    if (now > request_.lease_deadline) {
        return abort_with("client_lease_expired", "client lease expired", feedback, now);
    }

    if (seconds_between(started_, now) * 1000.0 > static_cast<double>(request_.timeout_ms)) {
        return abort_with("timeout", "motion timed out", feedback, now);
    }

    if (std::hypot(feedback.x - last_x_, feedback.y - last_y_) > kOdomJump) {
        update_progress(feedback);
        return abort_with("odom_jump", "odometry position jumped", feedback, now);
    }
    last_x_ = feedback.x;
    last_y_ = feedback.y;

    update_progress(feedback);
    const double elapsed = seconds_between(started_, now);

    if (correction_mode_) {
        if (seconds_between(correction_started_, now) >= kCorrectionTimeout) {
            return finish(false, "failed", "correction_timeout", "correction pass timed out");
        }
        if (request_.turn) {
            const double remaining = request_.angle_rad - yaw_accum_;
            if (std::abs(remaining) <= kAngleToleranceRad) {
                phase_ = Phase::SETTLING;
                settle_started_ = now;
                return MotionCommand{};
            }
            double yaw_cmd = clamp(kAngleKp * remaining, -kCorrectionMax, kCorrectionMax);
            if (std::abs(yaw_cmd) < kCorrectionMin) {
                yaw_cmd = std::copysign(kCorrectionMin, remaining);
            }
            return MotionCommand{0.0, 0.0, yaw_cmd};
        }
        const double rem_f = request_.forward_m - progress_forward_;
        const double rem_s = request_.strafe_m - progress_strafe_;
        const double rem_norm = std::hypot(rem_f, rem_s);
        if (rem_norm <= kPositionTolerance) {
            phase_ = Phase::SETTLING;
            settle_started_ = now;
            return MotionCommand{};
        }
        double cmd_f = kPositionKp * rem_f;
        double cmd_s = kPositionKp * rem_s;
        const double cmd_norm = std::hypot(cmd_f, cmd_s);
        if (cmd_norm > kCorrectionMax) {
            const double factor = kCorrectionMax / cmd_norm;
            cmd_f *= factor;
            cmd_s *= factor;
        } else if (cmd_norm < kCorrectionMin) {
            const double factor = kCorrectionMin / cmd_norm;
            cmd_f *= factor;
            cmd_s *= factor;
        }
        return MotionCommand{cmd_f, cmd_s, 0.0};
    }

    if (request_.turn) {
        const double remaining = request_.angle_rad - yaw_accum_;
        const double abs_remaining = std::abs(remaining);

        if (elapsed >= kDirectionWindow &&
            yaw_accum_ * direction_sign_ < -kDirectionAngleTolerance)
        {
            return abort_with(
                "direction_mismatch", "robot turned opposite to the command",
                feedback, now);
        }

        const double metric = std::abs(yaw_accum_);
        if (metric > last_progress_metric_ + 0.005) {
            last_progress_metric_ = metric;
            last_progress_ = now;
        } else if (elapsed >= 0.5 && seconds_between(last_progress_, now) >= kNoProgressWindow) {
            return abort_with("no_progress", "no yaw progress", feedback, now);
        }

        const double speed = std::abs(feedback.wz);
        if (abs_remaining <= kAngleToleranceRad + kPredictiveSeconds * speed) {
            phase_ = Phase::SETTLING;
            settle_started_ = now;
            return MotionCommand{};
        }

        double yaw_cmd = clamp(kAngleKp * remaining, -request_.max_speed_scale,
                               request_.max_speed_scale);
        if (abs_remaining > kMinCommandAngleRad &&
            std::abs(yaw_cmd) < kMinCommand)
        {
            yaw_cmd = std::copysign(kMinCommand, remaining);
        }
        return MotionCommand{0.0, 0.0, yaw_cmd};
    }

    const double target_f = request_.forward_m;
    const double target_s = request_.strafe_m;
    const double target_norm = std::hypot(target_f, target_s);
    const double rem_f = target_f - progress_forward_;
    const double rem_s = target_s - progress_strafe_;
    const double rem_norm = std::hypot(rem_f, rem_s);
    const double unit_f = target_f / target_norm;
    const double unit_s = target_s / target_norm;
    const double along = progress_forward_ * unit_f + progress_strafe_ * unit_s;

    if (elapsed >= kDirectionWindow && along < -kDirectionTolerance) {
        return abort_with(
            "direction_mismatch", "robot moved opposite to the command",
            feedback, now);
    }

    if (along > last_progress_metric_ + 0.005) {
        last_progress_metric_ = along;
        last_progress_ = now;
    } else if (elapsed >= 0.5 && seconds_between(last_progress_, now) >= kNoProgressWindow) {
        return abort_with("no_progress", "no position progress", feedback, now);
    }

    const double speed = std::hypot(feedback.vx, feedback.vy);
    if (rem_norm <= kPositionTolerance + kPredictiveSeconds * speed) {
        phase_ = Phase::SETTLING;
        settle_started_ = now;
        return MotionCommand{};
    }

    double cmd_f = kPositionKp * rem_f;
    double cmd_s = kPositionKp * rem_s;
    double cmd_norm = std::hypot(cmd_f, cmd_s);
    if (cmd_norm > request_.max_speed_scale) {
        const double factor = request_.max_speed_scale / cmd_norm;
        cmd_f *= factor;
        cmd_s *= factor;
        cmd_norm = request_.max_speed_scale;
    }
    if (rem_norm > kMinCommandDistance && cmd_norm < kMinCommand) {
        const double factor = kMinCommand / cmd_norm;
        cmd_f *= factor;
        cmd_s *= factor;
    }
    return MotionCommand{cmd_f, cmd_s, 0.0};
}

MotionResult MotionController::cancel(
    const std::string & code, const std::string & message,
    const MotionFeedback & feedback, TimePoint now)
{
    if (phase_ == Phase::IDLE || phase_ == Phase::DONE) {
        return result_;
    }
    abort_with(code, message, feedback, now);
    return result_;
}

void MotionController::reset()
{
    phase_ = Phase::IDLE;
    request_ = MotionRequest{};
    result_ = MotionResult{};
    progress_forward_ = 0.0;
    progress_strafe_ = 0.0;
    yaw_accum_ = 0.0;
    last_progress_metric_ = 0.0;
}

bool MotionController::active() const
{
    return phase_ == Phase::RUNNING || phase_ == Phase::SETTLING;
}

bool MotionController::settling() const
{
    return phase_ == Phase::SETTLING;
}

const MotionRequest & MotionController::request() const
{
    return request_;
}

MotionResult MotionController::result(
    const MotionFeedback & /*feedback*/, TimePoint /*now*/) const
{
    return result_;
}

double MotionController::progress_forward() const
{
    return progress_forward_;
}

double MotionController::progress_strafe() const
{
    return progress_strafe_;
}

double MotionController::progress_yaw_deg() const
{
    return yaw_accum_ * 180.0 / kPi;
}

double MotionController::elapsed_seconds(TimePoint now) const
{
    if (phase_ == Phase::IDLE || started_ == TimePoint{}) {
        return 0.0;
    }
    return std::chrono::duration<double>(now - started_).count();
}

long MotionController::lease_remaining_ms(TimePoint now) const
{
    if (phase_ == Phase::IDLE || phase_ == Phase::DONE) {
        return 0;
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        request_.lease_deadline - now).count();
    return remaining < 0 ? 0 : static_cast<long>(remaining);
}

void MotionController::extend_lease(TimePoint deadline)
{
    request_.lease_deadline = deadline;
}

}  // namespace agent_control_driver
