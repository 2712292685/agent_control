# agent_control — 自然语言控制 Zsibot 机器狗

用 ROS2 + Zsibot Genisom L1 SDK 控制机器狗，并把控制能力接入 Windows 端的聊天智能体 **AI-chatAgent**。用户在终端里说“请你前进 1 米”，智能体调用工具，驱动按里程计反馈闭环走到 1 米附近，再把实际位移回报给用户。

> **安全第一。** 本项目的“急停”是软件急停（`CMD_EMERGENCY_STOP`，机器狗会趴下/软停），**不是硬件急停**。实体遥控器、机器狗电源开关始终是最终安全手段。首次真机测试请在空旷场地、机器狗站稳、有人看护、实体遥控器在手的情况下，从 0.1–0.2 m / 10–20° 开始。
>
> 实现与自动化验证阶段**只使用 loopback 假机器狗**，不会向真机发送任何运动、姿态或急停指令。

## 架构

```text
Windows
  chatagent-robot (Python 3.12, DeepSeek function calling)
        │ HTTP/JSON  http://127.0.0.1:8100
        ▼
WSL2 / Linux
  agent_control_api (rclpy 生命周期 + 标准库 ThreadingHTTPServer + job manager)
        │ 本机 TCP 127.0.0.1:8766，行协议 + JSON 状态/结果
        ▼
  agent_control_driver (C++20 / rclcpp + Zsibot SDK, 30 Hz)
        │ UDP 192.168.234.1:8081
        ▼
     机器狗
```

- `agent_control_driver` 是唯一接触 SDK 的进程；**所有 `SetRemote`/`SetCmd` 只在 30 Hz 控制循环线程调用**。TCP 线程只解析字节、入队、非阻塞发送状态。
- `agent_control_api` 只做 HTTP ↔ 本机 TCP 的转换、任务记录和状态缓存，不依赖 ROS2 DDS 做控制传输；驱动仍是标准 ROS2 节点，发布 `/agent_control/status`。
- Windows agent 只依赖 HTTP；机器狗 SDK 是 Linux 静态库，所以驱动/API 运行在 WSL2 或 Linux PC，聊天智能体运行在 Windows。Windows 通过 `127.0.0.1` 访问 WSL2 的 TCP 服务（mirrored 网络推荐）；失败时用 `AGENT_CONTROL_BASE_URL` 指向 WSL IP。
- 运动是**异步任务**：HTTP 立即返回 202 和 `request_id`；Windows 工具每 200 ms 轮询任务；API 只在 Windows 仍在轮询时向驱动续租（lease）。Windows 进程死掉或网络断开后，驱动约 1.5 s 内自动停车。

## 功能

- 闭环移动：前进/后退/左右平移指定米数；原地左转/右转指定角度。
- 姿态：站立、趴下、平衡站立、移动模式、锁定。
- 停止（取消当前运动，不锁存）与软件急停/恢复。
- 状态查询：连接、控制权、电量、模式、位置/姿态、故障、当前任务。
- ROS2 状态话题 `/agent_control/status`（JSON String，5 Hz）。
- 安全兜底：30 Hz 持续 `SetRemote`、运动超时、无进度/方向不符/里程计跳变中止、API 链路与客户端 lease、退出 5 次零包。

## 目录

```text
agent_control/
├── README.md
├── docs/
│   ├── protocol.md              # TCP/HTTP/状态/工具 schema
│   └── safety.md                # 安全边界、急停、真机交接
├── src/
│   ├── agent_control_driver/    # ament_cmake, C++20
│   └── agent_control_api/       # ament_python
├── tools/
│   ├── fake_robot.py            # loopback UDP 假机器狗（带里程计/物理）
│   ├── check_axis_log.py        # 断言线上 joystick 轴
│   ├── test_api.sh              # HTTP 冒烟测试
│   ├── test_e2e.py              # 端到端仿真测试
│   └── run_sim.sh               # 一键启动仿真
└── windows/
    └── README.md                # Windows chatagent 接入说明
```

## 构建

```bash
source /opt/ros/humble/setup.bash
cd /home/snail/project/Development-course/agent_control
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
```

SDK 默认路径为相对 `agent_control/` 的 `../zsibot-sdk/genisom_L1_sdk`；如路径/架构不同：

```bash
colcon build --packages-select agent_control_driver --cmake-args \
  -DZSIBOT_SDK_DIR=/绝对路径/genisom_L1_sdk -DZSIBOT_ARCH=x86_64
```

## 仿真验证（不连真机）

使用 **18081/18080**，刻意避开真机默认的 8081/8080：

```bash
# 终端 1：先启假机器狗（解开 SDK 构造函数阻塞）
python3 -I tools/fake_robot.py \
  --listen 127.0.0.1:18081 --reply 127.0.0.1:18080 \
  --model XG --initial-function remote \
  --duration-sec 600 --log-file /tmp/agent_fake.jsonl \
  --forward-speed 0.8 --strafe-speed 0.5 --yaw-rate 1.2

# 终端 2：驱动 + API
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch agent_control_api agent_control.launch.py \
  robot_ip:=127.0.0.1 send_port:=18081 recv_port:=18080 \
  role:=sdk bridge_port:=8766 port:=8100 bind_address:=127.0.0.1

# 终端 3：测试
./tools/test_api.sh
python3 tools/test_e2e.py --fake-log /tmp/agent_fake.jsonl
```

也可以一键启动仿真栈：

```bash
./tools/run_sim.sh          # 另开终端跑 test_api.sh / test_e2e.py
```

预期：

- 驱动日志出现 TCP 控制桥监听、取得 SDK 控制权。
- API 打印 `http://127.0.0.1:8100/`。
- `/api/health` 返回 `ok:true`，`/api/status` 显示 `connected/in_control/motion_ready/odom_ready=true`。
- 假机器狗约 30 个 `remote`/秒；收到 `cmd 179` 后 `function=sdk`。
- `test_api.sh` 18/18；`test_e2e.py` 全部通过（前进/后退/平移/转向、姿态、停止、急停/恢复、边界、lease、轴映射）。
- 停止 launch 时假机器狗观察到 5 个零 `remote` 包。

## 真机运行（用户后续执行，不在本次自动化验证范围）

1. 机器狗上电、站稳，PC 连接机器狗热点；确认 PC 能访问 `192.168.234.0/24`。
2. 启动：
   ```bash
   ros2 launch agent_control_api agent_control.launch.py \
     robot_ip:=192.168.234.1 send_port:=8081 recv_port:=8080 role:=sdk
   ```
3. 检查 `/api/status`：`connected=true`、`in_control=true`、`motion_ready=true`、`estop_latched=false`。
4. Windows 端 `chatagent-robot` 的 `[robot].base_url` 默认 `http://127.0.0.1:8100`；若 API 在另一台 Linux PC，设置 `AGENT_CONTROL_BASE_URL=http://<PC_IP>:8100`。
5. 先站立、慢速，再做 0.1–0.2 m / 10–20° 的单轴测试；方向不对时先停下，调整 `sdk_forward_sign` / `sdk_strafe_sign` / `sdk_yaw_sign` / `yaw_feedback_sign`，不要改代码。
6. 测试停止、急停、恢复；在低速度下测试“驱动被 kill 时机器狗是否停下”。固件命令超时行为未经真机验证，实体遥控器必须始终在手。

> 真机与仿真不能同时运行：`read_state`、`control_demo`、`agent_control` 都会独占 SDK 的 UDP 8080。先停止一个再启动另一个。

## HTTP API

默认 `127.0.0.1:8100`。所有 POST 为 JSON；运动是异步任务。

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/health` | 驱动连接且状态新鲜 → 200；否则 503 |
| GET | `/api/status` | 状态 + `stale`/`driver_status_age_ms` |
| GET | `/api/jobs/<request_id>` | 任务记录；Windows 每 200 ms 轮询并刷新 lease |
| POST | `/api/move` | `{direction, distance_m, request_id?, max_speed_scale?, timeout_s?}` → 202 + job |
| POST | `/api/turn` | `{direction, angle_deg, ...}` → 202 + job |
| POST | `/api/command` | `{command: stand_up/sit_down/balance_stand/move_mode/lock}` → 202 + job |
| POST | `/api/stop` | 取消当前任务 → 202 |
| POST | `/api/estop` | 软件急停并锁存 → 202 |
| POST | `/api/estop/clear` | 恢复（会重新站立）→ 202 |

任务终态：`succeeded` / `failed` / `aborted` / `timed_out`。常见错误码：`busy`、`not_ready`、`estop_latched`、`not_latched`、`client_lease_expired`、`no_progress`、`direction_mismatch`、`odom_jump`、`timeout`、`interrupted`。

如果 `bind_address` 不是回环地址，必须设置环境变量 `AGENT_CONTROL_TOKEN`，否则 API 拒绝启动；客户端发送 `Authorization: Bearer <token>`。token 只从环境变量读取，不写配置文件或日志。

详细的 TCP、状态 JSON、任务和工具 schema 见 [docs/protocol.md](docs/protocol.md)。

## 轴映射（实测，重要）

用户实测：`remote_control.cpp` 注释写 q/e 转向、a/d 平移，但实际 **a/d 原地转向、q/e 左右平移**。因此 SDK API 顺序是：

```text
[0] forward  正值 = 前进
[1] strafe   正值 = 左平移
[2] yaw      正值 = 左转
[3] head     本项目固定 0
```

SDK 打包器在线上把四轴取反并交换前两轴：

```text
API  [forward, strafe, yaw, head]
wire [-strafe, -forward, -yaw, -head]
```

驱动用 `axis_mapping.hpp` 固定该顺序；假机器狗按同一顺序解码。若真机方向相反，只调整符号参数。`control_demo` 也已在 SDK 边界修正同样的映射。

## 主要参数

驱动：

| 参数 | 默认 | 说明 |
|---|---|---|
| `robot_ip` | `192.168.234.1` | 机器狗 IP |
| `send_port` / `recv_port` | `8081` / `8080` | SDK UDP 端口；仿真用 18081/18080 |
| `role` | `sdk` | `sdk` 或 `remote` |
| `control_rate_hz` | `30.0` | `SetRemote` 频率 |
| `status_rate_hz` | `5.0` | 状态频率 |
| `max_speed_scale` | `0.35` | 最大速度比例，0.05–0.6 |
| `bridge_port` | `8766` | 本机 TCP 桥端口 |
| `motion_timeout_ms` | `35000` | 单任务超时 |
| `lease_ms` | `1500` | 驱动运动 lease |
| `api_link_timeout_ms` | `1000` | API 静默超时 |
| `position_tolerance_m` / `angle_tolerance_deg` | `0.03` / `2.0` | 严格容差 |
| `acceptance_m` / `acceptance_deg` | `0.05` / `5.0` | 成功判定 |
| `no_progress_timeout_ms` | `2000` | 无进度超时 |
| `settle_ms` | `600` | 停车后 settle |
| `force_slow_speed` | `true` | 取得控制权后发一次 174 慢速 |
| `sdk_forward_sign` / `sdk_strafe_sign` / `sdk_yaw_sign` | `1.0` | 轴符号，真机校准用 |
| `yaw_feedback_sign` | `1.0` | 里程计 yaw 符号 |
| `max_forward_m` / `max_strafe_m` / `max_turn_deg` | `5.0` / `3.0` / `180.0` | 单任务硬限制 |

API：

| 参数 | 默认 | 说明 |
|---|---|---|
| `port` | `8100` | HTTP 端口 |
| `bind_address` | `127.0.0.1` | 非回环时必须设置 `AGENT_CONTROL_TOKEN` |
| `bridge_host` / `bridge_port` | `127.0.0.1` / `8766` | 驱动 TCP 桥 |
| `status_timeout_sec` | `2.0` | 状态过期判定 |
| `job_timeout_sec` | `45.0` | API 任务超时 |
| `poll_lease_timeout_sec` | `1.5` | 客户端停止轮询后停止续租 |

## 排错

| 现象 | 处理 |
|---|---|
| API 连接被拒绝 | 检查 launch 是否启动 API；`port` 是否被占用；WSL2 网络 |
| `/api/health` 503 | 驱动未连接/状态过期；检查 fake robot、SDK 端口、launch 终端 |
| `motion_ready=false` | 未取得控制权、未站稳/锁定、里程计未就绪、API 链路/lease 问题；看 `motion_ready_reason` |
| 运动返回 `not_ready` | 先 `stand_up` / `balance_stand` / `move_mode`；趴下/锁定状态不能移动 |
| 运动返回 `busy` | 已有任务；先 `/api/stop` |
| `client_lease_expired` | Windows 工具停止轮询（进程崩溃/网络断开）；检查客户端 |
| 急停后无法恢复 | 检查 90 重发、控制权、`cmd 122`；5 s 超时保持锁定 |
| `model is empty, please wait..` | SDK 在等 model；检查 fake robot/真机、IP、端口 |
| 方向不对 | 停！调整 `sdk_*_sign` / `yaw_feedback_sign`，不要直接改代码 |
| 端口/进程残留 | 用 `ss -ltnp \| grep -E ':(8100\|8766)'` 查 PID 并停止 |

## 已知限制

- 软件急停依赖控制权/网络，可能被实体遥控器抢占或链路中断；实体遥控器/断电是最终安全手段。
- 闭环精度依赖里程计；真机里程计漂移/不可用时驱动会报错中止，不会盲目按时间开环。
- 驱动被 kill 时机器狗固件的命令超时行为未在真机验证；必须保留实体遥控器。
- 不包含图传、建图、导航、特技动作。
- 网页/HTTP API 默认只监听回环；非回环必须带 token，且只应在可信热点使用。

## 许可证

MIT。
