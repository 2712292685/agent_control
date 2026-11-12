#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "zsibot_sdk/zsibot_api.h"

namespace
{

void usage()
{
    std::cout
        << "usage: sdk_axis_probe --axis N --value V [--robot-ip 127.0.0.1]\n"
        << "       [--send-port 18081] [--recv-port 18080] [--duration-ms 800]\n"
        << "axis: 0=forward 1=strafe 2=yaw 3=head\n";
}

}  // namespace

int main(int argc, char ** argv)
{
    int axis = 0;
    float value = 0.5f;
    std::string robot_ip = "127.0.0.1";
    int send_port = 18081;
    int recv_port = 18080;
    int duration_ms = 800;

    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        auto next = [&](const char * name) -> std::string {
            if (index + 1 >= argc) {
                std::cerr << "missing value for " << name << "\n";
                std::exit(2);
            }
            return argv[++index];
        };
        if (arg == "--axis") {
            axis = std::stoi(next("--axis"));
        } else if (arg == "--value") {
            value = std::stof(next("--value"));
        } else if (arg == "--robot-ip") {
            robot_ip = next("--robot-ip");
        } else if (arg == "--send-port") {
            send_port = std::stoi(next("--send-port"));
        } else if (arg == "--recv-port") {
            recv_port = std::stoi(next("--recv-port"));
        } else if (arg == "--duration-ms") {
            duration_ms = std::stoi(next("--duration-ms"));
        } else if (arg == "--help" || arg == "-h") {
            usage();
            return 0;
        } else {
            std::cerr << "unknown argument: " << arg << "\n";
            usage();
            return 2;
        }
    }
    if (axis < 0 || axis > 3) {
        std::cerr << "axis must be 0..3\n";
        return 2;
    }

    try {
        zsibot::ZsibotExecutor executor(
            zsibot::Role::ROLE_REMOTE, robot_ip, send_port, recv_port);
        std::array<float, 4> axes{0.0f, 0.0f, 0.0f, 0.0f};
        axes[static_cast<std::size_t>(axis)] = value;
        const auto end = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(duration_ms);
        while (std::chrono::steady_clock::now() < end) {
            executor.SetRemote(axes, std::array<float, 14>{});
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
        axes = {0.0f, 0.0f, 0.0f, 0.0f};
        for (int count = 0; count < 5; ++count) {
            executor.SetRemote(axes, std::array<float, 14>{});
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::cout << "sent axis " << axis << " value " << value << " for "
                  << duration_ms << " ms\n";
    } catch (const std::exception & exception) {
        std::cerr << "axis probe failed: " << exception.what() << "\n";
        return 1;
    }
    return 0;
}
