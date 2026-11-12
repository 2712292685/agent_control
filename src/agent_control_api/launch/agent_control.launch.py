from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _arg(name, default, description):
    return DeclareLaunchArgument(name, default_value=str(default), description=description)


def generate_launch_description():
    args = [
        _arg("robot_ip", "192.168.234.1", "机器狗 IP"),
        _arg("send_port", 8081, "SDK UDP 发送端口"),
        _arg("recv_port", 8080, "SDK UDP 接收端口"),
        _arg("role", "sdk", "sdk 或 remote"),
        _arg("control_rate_hz", 30.0, "SetRemote 频率"),
        _arg("status_rate_hz", 5.0, "状态频率"),
        _arg("max_speed_scale", 0.35, "默认最大速度比例"),
        _arg("bridge_port", 8766, "驱动 TCP 桥端口"),
        _arg("motion_timeout_ms", 35000, "单任务超时"),
        _arg("lease_ms", 1500, "驱动 lease"),
        _arg("force_slow_speed", "true", "取得控制权后发送慢速档"),
        _arg("sdk_forward_sign", 1.0, "前进轴符号"),
        _arg("sdk_strafe_sign", 1.0, "平移轴符号"),
        _arg("sdk_yaw_sign", 1.0, "转向轴符号"),
        _arg("yaw_feedback_sign", 1.0, "里程计 yaw 符号"),
        _arg("port", 8100, "HTTP API 端口"),
        _arg("bind_address", "127.0.0.1", "HTTP 监听地址"),
        _arg("status_timeout_sec", 2.0, "驱动状态过期判定"),
        _arg("job_timeout_sec", 45.0, "API 任务超时"),
        _arg("poll_lease_timeout_sec", 1.5, "客户端轮询 lease 超时"),
    ]

    driver = Node(
        package="agent_control_driver",
        executable="agent_control_driver",
        name="agent_control_driver",
        output="screen",
        parameters=[
            {
                "robot_ip": LaunchConfiguration("robot_ip"),
                "send_port": ParameterValue(
                    LaunchConfiguration("send_port"), value_type=int
                ),
                "recv_port": ParameterValue(
                    LaunchConfiguration("recv_port"), value_type=int
                ),
                "role": LaunchConfiguration("role"),
                "control_rate_hz": ParameterValue(
                    LaunchConfiguration("control_rate_hz"), value_type=float
                ),
                "status_rate_hz": ParameterValue(
                    LaunchConfiguration("status_rate_hz"), value_type=float
                ),
                "max_speed_scale": ParameterValue(
                    LaunchConfiguration("max_speed_scale"), value_type=float
                ),
                "bridge_port": ParameterValue(
                    LaunchConfiguration("bridge_port"), value_type=int
                ),
                "motion_timeout_ms": ParameterValue(
                    LaunchConfiguration("motion_timeout_ms"), value_type=int
                ),
                "lease_ms": ParameterValue(
                    LaunchConfiguration("lease_ms"), value_type=int
                ),
                "force_slow_speed": ParameterValue(
                    LaunchConfiguration("force_slow_speed"), value_type=bool
                ),
                "sdk_forward_sign": ParameterValue(
                    LaunchConfiguration("sdk_forward_sign"), value_type=float
                ),
                "sdk_strafe_sign": ParameterValue(
                    LaunchConfiguration("sdk_strafe_sign"), value_type=float
                ),
                "sdk_yaw_sign": ParameterValue(
                    LaunchConfiguration("sdk_yaw_sign"), value_type=float
                ),
                "yaw_feedback_sign": ParameterValue(
                    LaunchConfiguration("yaw_feedback_sign"), value_type=float
                ),
            }
        ],
    )

    api = Node(
        package="agent_control_api",
        executable="agent_control_api",
        name="agent_control_api",
        output="screen",
        parameters=[
            {
                "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                "bind_address": LaunchConfiguration("bind_address"),
                "bridge_host": "127.0.0.1",
                "bridge_port": ParameterValue(
                    LaunchConfiguration("bridge_port"), value_type=int
                ),
                "status_timeout_sec": ParameterValue(
                    LaunchConfiguration("status_timeout_sec"), value_type=float
                ),
                "job_timeout_sec": ParameterValue(
                    LaunchConfiguration("job_timeout_sec"), value_type=float
                ),
                "poll_lease_timeout_sec": ParameterValue(
                    LaunchConfiguration("poll_lease_timeout_sec"), value_type=float
                ),
            }
        ],
    )

    return LaunchDescription([*args, driver, api])
