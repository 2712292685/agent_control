# Windows 端 chatagent-robot

聊天智能体的代码在 Windows 本地仓库：

```text
D:\study\Files\AI-chatAgent
```

本次改动在保留原 `chatagent` 的前提下，新增：

- `chatagent-robot` 入口；
- `[robot]` 配置段；
- `robot_client.py` / `robot_context.py` / `robot_llm.py` / `robot_tools.py` / `robot_session.py` / `robot_policy.py` / `robot_cli.py`；
- 对应测试与文档。

## 启动顺序

1. 在 WSL2/Linux 启动机器狗驱动 + HTTP API（见 `agent_control/README.md`）：

   ```bash
   cd /home/snail/project/Development-course/agent_control
   ./tools/run_sim.sh      # 仿真；真机用 ros2 launch ...
   ```

2. 在 Windows PowerShell：

   ```powershell
   cd D:\study\Files\AI-chatAgent
   $env:DEEPSEEK_API_KEY = "sk-..."
   uv sync
   uv run chatagent-robot
   ```

   要安装成全局命令：

   ```powershell
   uv tool install --force .
   ```

   之后任意目录可运行 `chatagent-robot`；原 `chatagent` 仍可用。

## 连接 API

默认：

```toml
[robot]
base_url = "http://127.0.0.1:8100"
timeout_seconds = 55
max_tool_rounds = 4
```

- Windows 与 WSL2 在同一台机器、mirrored 网络时，`127.0.0.1:8100` 通常可用。
- 如果 API 在另一台 Linux PC，或 localhost 转发不可用：

  ```powershell
  $env:AGENT_CONTROL_BASE_URL = "http://192.168.234.44:8100"
  ```

- 如果 API 绑定了非回环地址，必须同时设置：

  ```powershell
  $env:AGENT_CONTROL_TOKEN = "<与 API 启动时相同的 token>"
  ```

  token 只从环境变量读取，不写入 `config.toml`。

## 示例

启动后在 `你 › ` 输入：

- `请你前进 1 米`
- `后退半米`
- `向左平移 0.2 米`
- `原地左转 90 度`
- `站起来`
- `趴下`
- `停下`
- `急停`
- `恢复控制`（CLI 会再次询问 y/N）
- `现在电量多少`

工具执行时终端会显示 `⚙ robot_move ...`；结束后显示实际位移/角度或错误。Ctrl+C 会请求 `/api/stop` 并结束当前动作。

## 安全

- 仿真阶段只连接 loopback 假机器狗，不连接真机。
- 真机首次测试必须低速、小幅度、实体遥控器在手；见 `agent_control/docs/safety.md`。
- `机器人急停`是软件急停，不是硬件急停。
- 网页/HTTP API 无账号体系；只在可信热点使用。
