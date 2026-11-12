#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>

#include "agent_control_driver/motion_controller.hpp"

using agent_control_driver::MotionCommand;
using agent_control_driver::MotionController;
using agent_control_driver::MotionFeedback;
using agent_control_driver::MotionRequest;
using agent_control_driver::MotionResult;
using agent_control_driver::TimePoint;

namespace
{

constexpr double kPi = 3.14159265358979323846;

MotionResult drive(
    MotionController & controller, MotionRequest request, const std::string & mode)
{
    TimePoint now = std::chrono::steady_clock::now();
    MotionFeedback feedback;
    feedback.finite = true;

    std::string error;
    if (!controller.start(request, feedback, now, error)) {
        std::cerr << "start failed: " << error << "\n";
        std::abort();
    }

    for (int tick = 0; tick < 2000; ++tick) {
        const MotionCommand command = controller.step(feedback, now);
        if (controller.result(feedback, now).finished) {
            return controller.result(feedback, now);
        }
        const double dt = 0.02;
        if (mode == "move") {
            feedback.x += command.forward * 0.5 * dt;
            feedback.vx = command.forward * 0.5;
        } else {
            feedback.yaw += command.yaw * 1.0 * dt;
            feedback.wz = command.yaw * 1.0;
        }
        now += std::chrono::milliseconds(20);
    }
    return controller.result(feedback, now);
}

}  // namespace

int main()
{
    {
        MotionController controller;
        MotionRequest request;
        request.request_id = "move-test";
        request.turn = false;
        request.forward_m = 0.5;
        request.max_speed_scale = 0.4;
        request.timeout_ms = 15000;
        request.lease_ms = 15000;
        const MotionResult result = drive(controller, request, "move");
        assert(result.finished);
        assert(result.success);
        assert(std::abs(result.forward_m - 0.5) <= 0.05);
        std::cout << "move result: " << result.forward_m << " m\n";
    }

    {
        MotionController controller;
        MotionRequest request;
        request.request_id = "turn-test";
        request.turn = true;
        request.angle_rad = kPi / 4.0;
        request.max_speed_scale = 0.4;
        request.timeout_ms = 15000;
        request.lease_ms = 15000;
        const MotionResult result = drive(controller, request, "turn");
        assert(result.finished);
        assert(result.success);
        assert(std::abs(result.yaw_deg - 45.0) <= 5.0);
        std::cout << "turn result: " << result.yaw_deg << " deg\n";
    }

    {
        MotionController controller;
        MotionRequest request;
        request.request_id = "cancel-test";
        request.turn = false;
        request.forward_m = 1.0;
        request.max_speed_scale = 0.4;
        request.timeout_ms = 15000;
        request.lease_ms = 15000;
        TimePoint now = std::chrono::steady_clock::now();
        MotionFeedback feedback;
        feedback.finite = true;
        std::string error;
        assert(controller.start(request, feedback, now, error));
        const MotionCommand command = controller.step(feedback, now);
        assert(std::abs(command.forward) > 0.0);
        const MotionResult result = controller.cancel(
            "interrupted", "test cancel", feedback, now);
        assert(result.finished);
        assert(result.state == "aborted");
        std::cout << "cancel result: " << result.state << "\n";
    }

    return 0;
}
