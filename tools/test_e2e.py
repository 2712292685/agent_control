#!/usr/bin/env python3
"""End-to-end simulation checks against a running agent_control stack.

Start the fake robot, driver and API first (see tools/run_sim.sh), then run:

    python3 tools/test_e2e.py --base-url http://127.0.0.1:8100 \
        --fake-log /tmp/agent_fake.jsonl

No real robot is touched; all addresses are loopback.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request

TERMINAL = {"succeeded", "failed", "aborted", "timed_out"}
PASS = 0
FAIL = 0


def check(name: str, condition: bool, detail: object = "") -> None:
    global PASS, FAIL
    if condition:
        print(f"PASS  {name}")
        PASS += 1
    else:
        print(f"FAIL  {name} -> {detail}")
        FAIL += 1


class Api:
    def __init__(self, base: str) -> None:
        self.base = base.rstrip("/")

    def get(self, path: str) -> tuple[int, dict]:
        try:
            with urllib.request.urlopen(self.base + path, timeout=5) as response:
                return response.status, json.load(response)
        except urllib.error.HTTPError as error:
            try:
                return error.code, json.load(error)
            except ValueError:
                return error.code, {}

    def post(self, path: str, payload: dict) -> tuple[int, dict]:
        request = urllib.request.Request(
            self.base + path,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=5) as response:
                return response.status, json.load(response)
        except urllib.error.HTTPError as error:
            try:
                return error.code, json.load(error)
            except ValueError:
                return error.code, {}

    def status(self) -> dict:
        _, body = self.get("/api/status")
        return body.get("driver") or {}

    def wait_job(self, request_id: str, timeout: float = 25.0) -> dict:
        deadline = time.time() + timeout
        job: dict = {}
        while time.time() < deadline:
            _, body = self.get("/api/jobs/" + request_id)
            job = body.get("job") or {}
            if job.get("state") in TERMINAL:
                return job
            time.sleep(0.2)
        return job

    def motion(self, path: str, payload: dict, timeout: float = 25.0) -> tuple[int, dict]:
        status, body = self.post(path, payload)
        if status >= 300 or "request_id" not in body:
            return status, body
        return status, self.wait_job(body["request_id"], timeout)

    def wait_until(self, predicate, timeout: float = 8.0) -> dict:
        deadline = time.time() + timeout
        driver: dict = {}
        while time.time() < deadline:
            driver = self.status()
            if predicate(driver):
                return driver
            time.sleep(0.2)
        return driver


def run(api: Api) -> None:
    status, body = api.get("/api/health")
    check("health 200", status == 200 and body.get("ok") is True, body)

    status, body = api.get("/api/status")
    driver = body.get("driver") or {}
    check("status connected/in_control", status == 200 and driver.get("connected") and driver.get("in_control"), driver)
    check("status motion_ready/odom_ready", driver.get("motion_ready") and driver.get("odom_ready"), driver)
    check("status axis map", driver.get("axis_map") == {"forward": 0, "strafe": 1, "yaw": 2, "head": 3}, driver.get("axis_map"))

    # Basic motions. These also generate the wire-axis evidence checked below.
    _, job = api.motion("/api/move", {"direction": "forward", "distance_m": 0.3})
    check("forward 0.3", job.get("state") == "succeeded", job)
    forward_m = float((job.get("result") or {}).get("forward_m", 0.0))
    check("forward distance", 0.25 <= forward_m <= 0.35, forward_m)

    _, job = api.motion("/api/move", {"direction": "backward", "distance_m": 0.2})
    check("backward 0.2", job.get("state") == "succeeded", job)

    _, job = api.motion("/api/move", {"direction": "left", "distance_m": 0.2})
    check("left strafe 0.2", job.get("state") == "succeeded", job)
    left = float((job.get("result") or {}).get("strafe_m", 0.0))
    check("left strafe sign", left > 0.0, left)

    _, job = api.motion("/api/move", {"direction": "right", "distance_m": 0.2})
    check("right strafe 0.2", job.get("state") == "succeeded", job)
    right = float((job.get("result") or {}).get("strafe_m", 0.0))
    check("right strafe sign", right < 0.0, right)

    _, job = api.motion("/api/turn", {"direction": "left", "angle_deg": 45})
    check("turn left 45", job.get("state") == "succeeded", job)
    yaw = float((job.get("result") or {}).get("yaw_deg", 0.0))
    check("turn left sign", yaw > 0.0, yaw)

    _, job = api.motion("/api/turn", {"direction": "right", "angle_deg": 30})
    check("turn right 30", job.get("state") == "succeeded", job)
    yaw = float((job.get("result") or {}).get("yaw_deg", 0.0))
    check("turn right sign", yaw < 0.0, yaw)

    # Validation.
    for name, path, payload, expected in [
        ("distance 0", "/api/move", {"direction": "forward", "distance_m": 0.0}, 422),
        ("distance 6", "/api/move", {"direction": "forward", "distance_m": 6.0}, 422),
        ("lateral 4", "/api/move", {"direction": "left", "distance_m": 4.0}, 422),
        ("angle 0", "/api/turn", {"direction": "left", "angle_deg": 0}, 422),
        ("angle 181", "/api/turn", {"direction": "left", "angle_deg": 181}, 422),
        ("bad direction", "/api/move", {"direction": "up", "distance_m": 0.2}, 400),
        ("bool distance", "/api/move", {"direction": "forward", "distance_m": True}, 400),
    ]:
        code, _ = api.post(path, payload)
        check(name, code == expected, code)

    # Posture. Sit/lock must reject motion; balance recovers a movable state.
    for command in ("stand_up", "sit_down", "balance_stand", "move_mode", "lock"):
        code, job = api.motion("/api/command", {"command": command})
        check(f"posture {command}", code == 202 and job.get("state") == "succeeded", job)

    code, _ = api.post("/api/move", {"direction": "forward", "distance_m": 0.1})
    check("move rejected while locked", code == 409, code)
    _, job = api.motion("/api/command", {"command": "balance_stand"})
    check("balance after lock", job.get("state") == "succeeded", job)

    # Stop during motion.
    code, body = api.post("/api/move", {"direction": "forward", "distance_m": 2.0})
    check("stop test start", code == 202, body)
    request_id = (body or {}).get("request_id", "")
    time.sleep(0.4)
    code, _ = api.post("/api/stop", {})
    check("stop request", code == 202, code)
    job = api.wait_job(request_id)
    check("stopped job aborted", job.get("state") == "aborted", job)
    moved = float((job.get("result") or {}).get("forward_m", 0.0))
    check("stopped before target", 0.0 < moved < 2.0, moved)
    time.sleep(0.5)

    # E-stop and recovery.
    code, body = api.post("/api/move", {"direction": "forward", "distance_m": 2.0})
    request_id = (body or {}).get("request_id", "")
    time.sleep(0.4)
    code, _ = api.post("/api/estop", {})
    check("estop request", code == 200, code)
    job = api.wait_job(request_id)
    check("estop job aborted", job.get("state") == "aborted", job)
    driver = api.wait_until(lambda value: bool(value.get("estop_latched")), 5.0)
    check("estop latched", driver.get("estop_latched") is True, driver)
    code, _ = api.post("/api/estop/clear", {})
    check("estop clear request", code == 200, code)
    driver = api.wait_until(
        lambda value: not value.get("estop_latched") and value.get("motion_ready"), 8.0
    )
    check("estop recovered", not driver.get("estop_latched") and driver.get("motion_ready"), driver)

    # Client lease: submit a long move and never poll its job.
    code, body = api.post("/api/move", {"direction": "forward", "distance_m": 2.0})
    request_id = (body or {}).get("request_id", "")
    check("lease test start", code == 202 and bool(request_id), body)
    time.sleep(4.0)
    _, body = api.get("/api/jobs/" + request_id)
    job = body.get("job") or {}
    check("lease expired", job.get("state") == "aborted" and (job.get("error") or {}).get("code") == "client_lease_expired", job)
    time.sleep(0.5)


def check_axis_log(path: str) -> None:
    vectors: set[tuple[float, float, float, float]] = set()
    try:
        with open(path, encoding="utf-8") as handle:
            for line in handle:
                try:
                    packet = json.loads(line).get("packet", {})
                except ValueError:
                    continue
                if packet.get("type") != "remote":
                    continue
                joystick = packet.get("joystick") or []
                if len(joystick) >= 4 and any(abs(float(value)) > 0.01 for value in joystick):
                    vectors.add(tuple(round(float(value), 3) for value in joystick[:4]))
    except OSError as error:
        check("axis log readable", False, error)
        return
    check(
        "wire forward axis (index 1, negative)",
        any(abs(v[1]) > 0.01 and abs(v[0]) < 0.01 and abs(v[2]) < 0.01 for v in vectors),
        sorted(vectors)[:8],
    )
    check(
        "wire strafe axis (index 0, negative for left)",
        any(v[0] < -0.01 and abs(v[1]) < 0.01 and abs(v[2]) < 0.01 for v in vectors),
        sorted(vectors)[:8],
    )
    check(
        "wire yaw axis (index 2, negative for left)",
        any(v[2] < -0.01 and abs(v[0]) < 0.01 and abs(v[1]) < 0.01 for v in vectors),
        sorted(vectors)[:8],
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-url", default="http://127.0.0.1:8100")
    parser.add_argument("--fake-log", default="/tmp/agent_fake.jsonl")
    args = parser.parse_args()
    api = Api(args.base_url)
    try:
        run(api)
        check_axis_log(args.fake_log)
    except Exception as error:
        check("unexpected exception", False, repr(error))
    print(f"\nSummary: {PASS} passed, {FAIL} failed")
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
