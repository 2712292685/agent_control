# agent_control 协议

本文档是驱动、API、Windows 工具之间的唯一接口约定。实现变动时先改这里。

## 1. 驱动 ↔ API TCP（127.0.0.1:8766）

换行分隔 ASCII，单行 ≤512 B。API 是唯一客户端；断线 1 s 重连。

### API → 驱动

```text
PING
MOVE  <request_id> <forward_m> <strafe_m> <max_speed_scale> <timeout_ms> <lease_ms>
TURN  <request_id> <angle_rad> <max_speed_scale> <timeout_ms> <lease_ms>
CMD   <request_id> <cmd_code>
STOP  <request_id>
ESTOP <request_id>
ECLEAR <request_id>
KEEP  <request_id>
```

- `request_id`：`[A-Za-z0-9_-]{1,64}`。
- `forward_m`：正=前进，负=后退。
- `strafe_m`：正=左平移，负=右平移。
- `angle_rad`：正=左转，负=右转。
- `max_speed_scale`：驱动会再按 `max_speed_scale` 参数封顶。
- `lease_ms`：运动 lease；API 通过 `KEEP` 续租。
- `CMD` 仅接受 `122/106/138/154/155`（站立/趴下/移动/平衡/锁定）。
- `PING` 每 200 ms；驱动 1 s 没有收到任何行即认为 API 链路丢失。

### 驱动 → API

```text
A:{"request_id":"...","accepted":true,"duplicate":false,"code":"","message":""}
R:{"request_id":"...","kind":"move","state":"succeeded","result":{"forward_m":0.98,"strafe_m":0.0,"yaw_deg":0.0,"elapsed_ms":2140},"error":null,"elapsed_ms":2140}
S:{status json}
E:{"code":"bad_line","message":"..."}
```

- `A:` 是命令接受/拒绝。
- `R:` 是终态；重复 `request_id` 直接回缓存，不重复执行。
- `S:` 5 Hz 状态。
- 驱动在运动期间 1 s 无 API 行、或 lease 过期、或 TCP 断开，会取消运动并归零。

## 2. 状态 JSON

```json
{
  "schema_version": 1, "seq": 42, "link_ok": true,
  "executor_ready": true, "connected": true, "in_control": true,
  "role": "sdk", "model": "XG",
  "estop_latched": false, "estop_clear_pending": false,
  "motion_ready": true, "motion_ready_reason": "", "odom_ready": true,
  "function_mode": 3, "function_mode_name": "sdk",
  "control_mode": 6, "control_mode_name": "stay",
  "motion_mode": 2, "motion_mode_name": "rest",
  "motion_type": 7, "motion_type_name": "unknown",
  "battery": {"power": 86, "volt": 24.1, "current": 1.23, "temp": 30.5, "error": 0},
  "body_temp_c": 36.0, "speed_level": 1, "speed_level_name": "slow",
  "axis_map": {"forward": 0, "strafe": 1, "yaw": 2, "head": 3},
  "pose": {
    "position": [0.12, 0.01, 0.0],
    "rpy": [0.0, 0.0, 0.05],
    "yaw_unwrapped": 0.05,
    "body_velocity": [0.3, 0.0, 0.0],
    "world_velocity": [0.3, 0.0, 0.0]
  },
  "active_job": {
    "request_id": "a1b2", "kind": "move", "phase": "moving", "state": "running",
    "active": true,
    "requested": {"forward_m": 1.0, "strafe_m": 0.0, "angle_deg": 0.0},
    "measured": {"forward_m": 0.42, "strafe_m": 0.0, "yaw_deg": 0.0},
    "elapsed_ms": 1200, "lease_remaining_ms": 700, "error": null
  },
  "last_job": null,
  "faults": [], "last_error": "", "tx_hz": 30.1, "api_rx_hz": 5.0
}
```

`motion_ready_reason` 常见值：`not_connected`、`no_control`、`estop_latched`、`odometry_not_ready`、`wrong_function_mode`、`not_standing`、`posture_active`、`api_link_lost`。

## 3. HTTP API

默认 `http://127.0.0.1:8100`。POST 只接受 `application/json`，body ≤4 KiB。

### 健康与状态

```http
GET /api/health
GET /api/status
```

`/api/health` 在驱动连接且状态新鲜时返回 200，否则 503。`/api/status` 始终返回缓存（含 `stale`、`driver_status_age_ms`、`driver`）。

### 运动

```http
POST /api/move
{"direction":"forward","distance_m":1.0}
```

```http
POST /api/turn
{"direction":"left","angle_deg":90}
```

可选字段：`request_id`（重试时复用）、`max_speed_scale`（0.05–0.5）、`timeout_s`（1–30）。响应：

```http
HTTP/1.1 202 Accepted
{"ok":true,"request_id":"...","job":{"ok":true,"request_id":"...","kind":"move","state":"running","requested":{...},"result":null,"error":null}}
```

轮询：

```http
GET /api/jobs/<request_id>
```

终态：`succeeded` / `failed` / `aborted` / `timed_out`。`result` 包含实际 `forward_m`、`strafe_m`、`yaw_deg`、`elapsed_ms`。

### 姿态/停止/急停

```http
POST /api/command  {"command":"stand_up"}
POST /api/stop     {}
POST /api/estop    {}
POST /api/estop/clear {}
```

`/api/estop` 会等待驱动确认 `estop_latched=true`（最多 3 s）后返回 200；`/api/estop/clear` 会等待驱动恢复（最多 7 s）后返回 200。确认失败返回 504，工具应如实报告，不能假定成功。

姿态命令同样返回 202 + job；Windows 工具轮询到终态后再回答用户。

### 错误

```json
{"ok": false, "error": {"code": "busy", "message": "another job is active; stop it first"}}
```

部分参数错误返回 `{"ok":false,"error":"invalid_direction"}`。常见 HTTP 状态：400 参数格式、409 busy/not_ready、422 越界、503 驱动断开、504 超时。

### 鉴权

默认只监听回环，不需要 token。若 `bind_address` 不是回环，必须设置 `AGENT_CONTROL_TOKEN`，并发送：

```http
Authorization: Bearer <token>
```

## 4. Windows 工具 schema

| 工具 | 参数 | 说明 |
|---|---|---|
| `robot_status` | — | 状态查询 |
| `robot_move` | `direction: forward/backward/left/right`, `distance_m: 0.05–5` | 左右是平移 |
| `robot_turn` | `direction: left/right`, `angle_deg: 1–180` | 原地转向 |
| `robot_posture` | `command: stand_up/sit_down/balance_stand/move_mode/lock` | 姿态 |
| `robot_stop` | — | 取消当前运动 |
| `robot_emergency_stop` | — | 软件急停 |
| `robot_clear_estop` | — | 恢复（CLI 需二次确认） |

工具结果必须是 JSON 字符串，包含 `ok`、`request_id`、`state`、实际测量值和 `error`；不能伪造成功。

## 5. 幂等与 lease

- Windows 端为每次动作生成 UUID4 `request_id`；HTTP 超时后用同一个 id 轮询，不要生成新 id 重发。
- 驱动缓存最近 64 条终态，重复 `MOVE/TURN` 直接回缓存。
- API 的 job manager 按 `request_id` 去重。
- Windows 每 200 ms `GET /api/jobs/<id>`；API 只有在最近 1.5 s 内被轮询时才向驱动发 `KEEP`。Windows 进程死掉 → 停止轮询 → 驱动 lease 过期 → 停车。
