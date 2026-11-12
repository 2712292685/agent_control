#!/usr/bin/env python3
"""Loopback-only fake Zsibot robot for agent_control simulation.

The SDK sends heartbeats/remote/cmd to the fake's --listen address (default
127.0.0.1:18081) and receives on --reply (default 127.0.0.1:18080). Replies
must go to the fixed reply address, not the heartbeat source port.

The fake decodes the SDK wire joystick as:
    forward = -wire[1], strafe = -wire[0], yaw = -wire[2]

because ZsibotPacker negates all axes and swaps the first two. It integrates a
simple body-frame plant and publishes odom_info/imu_info so the driver's
closed-loop controller can be tested without a real robot.

This file is test-only. It refuses non-loopback addresses unless
--allow-non-loopback is given.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import signal
import socket
import sys
import threading
import time

MODEL_NAMES = {"XG", "XGW", "XGWHSPD"}
CONTROL_MOVING = {"stay", "stand_up"}


class FakeRobot:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.listen = args.listen
        self.reply = args.reply
        self.model = args.model
        self.function = args.initial_function
        self.control_mode = "stay"
        self.motion_mode = "rest"
        self.motion_type = "unknown"
        self.speed_level = "low"
        self.grant_at: float | None = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.start = time.monotonic()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx_body = 0.0
        self.vy_body = 0.0
        self.wz = 0.0
        self.last_joystick = [0.0, 0.0, 0.0, 0.0]
        self.last_remote = 0.0
        self.last_pose_jump = time.monotonic()
        self.noise = random.Random(7)

        self.counters = {"heartbeat": 0, "remote": 0, "cmd": 0}
        self.cmd_counts: dict[int, int] = {}
        self.last_summary_time = time.monotonic()
        self.last_remote_count = 0

        self.log_file = None
        if args.log_file:
            self.log_file = open(args.log_file, "w", encoding="utf-8")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(self.listen)
        self.sock.settimeout(0.2)

    # ---------------------------- packets ----------------------------
    def dog_state(self) -> dict:
        return {
            "type": "dog_state",
            "result": 0,
            "power": 86,
            "temp": 36.0,
            "sn": "SIM-XG-0001",
            "speed": self.vx_body,
            "angle_speed": self.wz,
            "shift_speed": self.vy_body,
            "angle": self.yaw,
            "dev_name": "FakeDog",
            "ssid": "Zsibot-SIM",
            "model": self.model,
        }

    def dev_info(self) -> dict:
        return {
            "type": "dev_info",
            "result": 0,
            "battery": {
                "current": 1230,
                "error": 0,
                "power": 86,
                "temp": 30500,
                "volt": 24100,
            },
        }

    def feedback(self) -> dict:
        return {
            "type": "feedback",
            "result": 0,
            "function": self.function,
            "control_mode": self.control_mode,
            "motion_mode": self.motion_mode,
            "motion_type": self.motion_type,
            "motor_temp": [30.0 + index * 0.5 for index in range(16)],
        }

    def imu_info(self) -> dict:
        half = self.yaw * 0.5
        return {
            "type": "imu_info",
            "result": 0,
            "acc": [0.0, 0.0, 9.81],
            "gyro": [0.0, 0.0, self.wz],
            "quat": [math.cos(half), 0.0, 0.0, math.sin(half)],
            "rpy": [0.0, 0.0, self.yaw],
        }

    def odom_info(self) -> dict:
        x = 0.0 if self.args.no_odom else self.x
        y = 0.0 if self.args.no_odom else self.y
        if self.args.noise_m > 0 and not self.args.no_odom:
            x += self.noise.gauss(0.0, self.args.noise_m)
            y += self.noise.gauss(0.0, self.args.noise_m)
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        vx_world = c * self.vx_body - s * self.vy_body
        vy_world = s * self.vx_body + c * self.vy_body
        return {
            "type": "odom_info",
            "result": 0,
            "v_world": [vx_world, vy_world, 0.0],
            "position": [x, y, 0.0],
            "omega_world": [0.0, 0.0, self.wz],
            "v_body": [self.vx_body, self.vy_body, 0.0],
            "omega_body": [0.0, 0.0, self.wz],
        }

    def leg_joint_info(self) -> dict:
        zeros = [0.0, 0.0, 0.0, 0.0]
        return {
            "type": "leg_joint_info",
            "result": 0,
            "leg_joint_info": {
                "leg_abad_joint": zeros,
                "leg_hip_joint": zeros,
                "leg_knee_joint": zeros,
                "leg_foot_joint": zeros,
                "leg_abad_joint_vel": zeros,
                "leg_hip_joint_vel": zeros,
                "leg_knee_joint_vel": zeros,
                "leg_foot_joint_vel": zeros,
                "leg_abad_joint_torque": zeros,
                "leg_hip_joint_torque": zeros,
                "leg_knee_joint_torque": zeros,
                "leg_foot_joint_torque": zeros,
            },
        }

    def version(self) -> dict:
        return {
            "type": "version",
            "result": 0,
            "version": {"mc": "SIM-MC-1.0", "dog_task": "SIM-TASK-1.0"},
        }

    def fault_info(self) -> dict:
        return {"type": "fault_info", "result": 0, "faults": []}

    def speed_set(self) -> dict:
        return {
            "type": "speed_set",
            "result": 0,
            "level": self.speed_level,
            "high": {"x": 1.0, "y": 0.5, "yaw": 1.0},
            "mid": {"x": 0.6, "y": 0.3, "yaw": 0.6},
            "low": {"x": 0.3, "y": 0.2, "yaw": 0.3},
        }

    # ---------------------------- threads ----------------------------
    def sender_loop(self) -> None:
        tick = 0
        fast_packets = [
            self.dog_state,
            self.dev_info,
            self.feedback,
            self.imu_info,
            self.odom_info,
            self.leg_joint_info,
        ]
        slow_packets = [self.version, self.fault_info, self.speed_set]
        while not self.stop_event.is_set():
            now = time.monotonic()
            with self.lock:
                if self.grant_at is not None and now >= self.grant_at:
                    self.function = "sdk"
                    self.grant_at = None
                    print("[fake] control granted: function=sdk", flush=True)
                packets = [builder() for builder in fast_packets]
                if tick % 10 == 0:
                    packets.extend(builder() for builder in slow_packets)
            for packet in packets:
                self.sock.sendto(
                    json.dumps(packet, separators=(",", ":")).encode("utf-8"),
                    self.reply,
                )
            tick += 1
            self.stop_event.wait(0.1)

    def physics_loop(self) -> None:
        dt = 0.02
        while not self.stop_event.is_set():
            now = time.monotonic()
            with self.lock:
                active = (
                    self.function == "sdk"
                    and self.control_mode in CONTROL_MOVING
                    and not self.args.stall
                    and now - self.last_remote <= 0.3
                )
                wire = self.last_joystick if active else [0.0, 0.0, 0.0, 0.0]
                forward = -float(wire[1])
                strafe = -float(wire[0])
                yaw_cmd = -float(wire[2])
                if self.args.invert_strafe:
                    strafe = -strafe
                if self.args.invert_yaw:
                    yaw_cmd = -yaw_cmd
                if active and not self.args.no_odom:
                    self.vx_body = forward * self.args.forward_speed
                    self.vy_body = strafe * self.args.strafe_speed
                    self.wz = yaw_cmd * self.args.yaw_rate
                    c = math.cos(self.yaw)
                    s = math.sin(self.yaw)
                    self.x += (c * self.vx_body - s * self.vy_body) * dt
                    self.y += (s * self.vx_body + c * self.vy_body) * dt
                    self.yaw += self.wz * dt
                    if self.yaw > math.pi:
                        self.yaw -= 2.0 * math.pi
                    elif self.yaw < -math.pi:
                        self.yaw += 2.0 * math.pi
                else:
                    self.vx_body = 0.0
                    self.vy_body = 0.0
                    self.wz = 0.0
                if self.args.pose_jump and not self.args.no_odom:
                    if now - self.last_pose_jump >= 2.0:
                        self.last_pose_jump = now
                        self.x += 1.0
                        print("[fake] pose jump +1.0 m", flush=True)
            self.stop_event.wait(dt)

    def receive_loop(self) -> None:
        last_summary = time.monotonic()
        while not self.stop_event.is_set():
            try:
                data, _addr = self.sock.recvfrom(65535)
            except socket.timeout:
                data = None
            except OSError:
                break

            if data:
                try:
                    packet = json.loads(data.decode("utf-8"))
                except (UnicodeDecodeError, ValueError):
                    packet = {
                        "type": "invalid",
                        "raw": data[:80].decode("utf-8", "replace"),
                    }
                self._handle_packet(packet)
                if self.log_file:
                    self.log_file.write(
                        json.dumps({"ts": time.time(), "packet": packet},
                                   ensure_ascii=False) + "\n"
                    )
                    self.log_file.flush()

            now = time.monotonic()
            if not self.args.quiet and now - last_summary >= 1.0:
                last_summary = now
                self._print_summary()

    def _handle_packet(self, packet: dict) -> None:
        packet_type = packet.get("type")
        now = time.monotonic()
        with self.lock:
            if packet_type == "heartbeat":
                self.counters["heartbeat"] += 1
            elif packet_type == "remote":
                self.counters["remote"] += 1
                joystick = packet.get("joystick", [])
                if isinstance(joystick, list) and len(joystick) >= 4:
                    self.last_joystick = [float(value) for value in joystick[:4]]
                    self.last_remote = now
            elif packet_type == "cmd":
                code = int(packet.get("cmd", 0))
                self.counters["cmd"] += 1
                self.cmd_counts[code] = self.cmd_counts.get(code, 0) + 1
                self._apply_command(code, now)

    def _apply_command(self, code: int, now: float) -> None:
        if code == 179:
            if self.function != "sdk" and self.grant_at is None:
                self.grant_at = now + self.args.control_grant_delay_ms / 1000.0
        elif code == 90:
            self.control_mode = "emergency"
            self.vx_body = self.vy_body = self.wz = 0.0
            print("[fake] cmd 90: EMERGENCY STOP", flush=True)
        elif code == 122:
            self.control_mode = "stand_up"
            print("[fake] cmd 122: stand up", flush=True)
        elif code == 106:
            self.control_mode = "stand_down"
            self.vx_body = self.vy_body = self.wz = 0.0
            print("[fake] cmd 106: sit down", flush=True)
        elif code == 138:
            self.control_mode = "stay"
            print("[fake] cmd 138: move mode", flush=True)
        elif code == 154:
            self.control_mode = "stay"
            print("[fake] cmd 154: balance stand", flush=True)
        elif code == 155:
            self.control_mode = "lock"
            self.vx_body = self.vy_body = self.wz = 0.0
            print("[fake] cmd 155: lock", flush=True)
        elif code == 174:
            self.speed_level = "low"
            print("[fake] cmd 174: slow", flush=True)
        elif code == 175:
            self.speed_level = "mid"
            print("[fake] cmd 175: normal", flush=True)
        elif code == 176:
            self.speed_level = "high"
            print("[fake] cmd 176: fast", flush=True)
        else:
            print(f"[fake] cmd {code} (unhandled)", flush=True)

    def _print_summary(self) -> None:
        now = time.monotonic()
        with self.lock:
            counters = dict(self.counters)
            joystick = list(self.last_joystick)
            function = self.function
            control = self.control_mode
            speed = self.speed_level
            commands = dict(self.cmd_counts)
            window = max(0.001, now - self.last_summary_time)
            rate = (counters["remote"] - self.last_remote_count) / window
            self.last_summary_time = now
            self.last_remote_count = counters["remote"]
        print(
            "[fake] hb=%d remote=%d (%.1f/s) cmd=%d joy=(%+.2f,%+.2f,%+.2f,%+.2f) "
            "function=%s control=%s speed=%s commands=%s"
            % (
                counters["heartbeat"],
                counters["remote"],
                rate,
                counters["cmd"],
                joystick[0],
                joystick[1],
                joystick[2],
                joystick[3],
                function,
                control,
                speed,
                commands,
            ),
            flush=True,
        )

    # ---------------------------- lifecycle ----------------------------
    def run(self) -> None:
        print(
            "[fake] listening on %s:%d, replying to %s:%d, model=%s, "
            "initial function=%s, stall=%s no_odom=%s"
            % (
                self.listen[0],
                self.listen[1],
                self.reply[0],
                self.reply[1],
                self.model,
                self.args.initial_function,
                self.args.stall,
                self.args.no_odom,
            ),
            flush=True,
        )
        sender = threading.Thread(target=self.sender_loop, daemon=True)
        receiver = threading.Thread(target=self.receive_loop, daemon=True)
        physics = threading.Thread(target=self.physics_loop, daemon=True)
        sender.start()
        receiver.start()
        physics.start()

        deadline = None
        if self.args.duration_sec > 0:
            deadline = time.monotonic() + self.args.duration_sec
        try:
            while not self.stop_event.is_set():
                if deadline is not None and time.monotonic() >= deadline:
                    break
                self.stop_event.wait(0.2)
        except KeyboardInterrupt:
            pass
        self.stop_event.set()
        sender.join(timeout=1.0)
        receiver.join(timeout=1.0)
        physics.join(timeout=1.0)
        self._print_summary()
        if self.log_file:
            self.log_file.close()
        self.sock.close()
        print("[fake] stopped", flush=True)


def parse_address(text: str) -> tuple[str, int]:
    host, _, port = text.rpartition(":")
    if not host or not port:
        raise argparse.ArgumentTypeError("address must be HOST:PORT")
    return host, int(port)


def main() -> int:
    parser = argparse.ArgumentParser(description="Fake Zsibot robot for agent_control")
    parser.add_argument("--listen", type=parse_address, default=("127.0.0.1", 18081))
    parser.add_argument("--reply", type=parse_address, default=("127.0.0.1", 18080))
    parser.add_argument("--model", choices=sorted(MODEL_NAMES), default="XG")
    parser.add_argument("--initial-function", choices=["remote", "sdk"], default="remote")
    parser.add_argument("--control-grant-delay-ms", type=int, default=200)
    parser.add_argument("--duration-sec", type=float, default=0.0)
    parser.add_argument("--log-file", default=None)
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--forward-speed", type=float, default=0.8)
    parser.add_argument("--strafe-speed", type=float, default=0.5)
    parser.add_argument("--yaw-rate", type=float, default=1.2)
    parser.add_argument("--noise-m", type=float, default=0.0)
    parser.add_argument("--stall", action="store_true")
    parser.add_argument("--no-odom", action="store_true")
    parser.add_argument("--invert-strafe", action="store_true")
    parser.add_argument("--invert-yaw", action="store_true")
    parser.add_argument("--pose-jump", action="store_true")
    parser.add_argument("--allow-non-loopback", action="store_true")
    args = parser.parse_args()

    if not args.allow_non_loopback:
        for address in (args.listen, args.reply):
            if address[0] not in ("127.0.0.1", "localhost"):
                parser.error("non-loopback address requires --allow-non-loopback")

    robot = FakeRobot(args)

    def stop(_signum: int, _frame: object) -> None:
        robot.stop_event.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    robot.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
