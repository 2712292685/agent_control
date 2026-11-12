"""HTTP/JSON API for the Windows chat agent.

The Windows process only needs HTTP. Motion is asynchronous: POST returns 202
and a job record, and the caller polls GET /api/jobs/<request_id>.
"""

from __future__ import annotations

import json
import math
import re
import time
import uuid
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from .job_manager import JobManager, public_record
from .driver_client import DriverClient

REQUEST_ID_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")
MAX_BODY = 4096

COMMAND_CODES = {
    "stand_up": 122,
    "sit_down": 106,
    "balance_stand": 154,
    "move_mode": 138,
    "lock": 155,
}

MOVE_DIRECTIONS = {"forward", "backward", "left", "right"}
TURN_DIRECTIONS = {"left", "right"}


class ControlHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        server_address: tuple[str, int],
        handler_class: type[BaseHTTPRequestHandler],
        jobs: JobManager,
        client: DriverClient,
        token: str = "",
    ) -> None:
        super().__init__(server_address, handler_class)
        self.jobs = jobs
        self.client = client
        self.token = token


class ControlRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "AgentControl/0.1"
    server: ControlHTTPServer

    def log_request(self, code: int | str = "-", size: int | str = "-") -> None:
        if isinstance(code, int) and code < 400:
            return
        super().log_request(code, size)

    # ---------------------------- GET ----------------------------
    def do_GET(self) -> None:
        if not self._authorized():
            self._reject(HTTPStatus.UNAUTHORIZED, "unauthorized")
            return
        path = urlparse(self.path).path
        if path == "/api/health":
            status = self.server.client.get_status()
            driver = status.get("driver") or {}
            healthy = (
                not status.get("stale", True)
                and status.get("bridge_connected", False)
                and bool(driver.get("executor_ready"))
            )
            self._send_json(
                HTTPStatus.OK if healthy else HTTPStatus.SERVICE_UNAVAILABLE,
                {
                    "ok": healthy,
                    "stale": status.get("stale"),
                    "bridge_connected": status.get("bridge_connected"),
                    "connected": driver.get("connected"),
                    "in_control": driver.get("in_control"),
                    "estop_latched": driver.get("estop_latched"),
                    "motion_ready": driver.get("motion_ready"),
                },
            )
            return
        if path == "/api/status":
            self._send_json(HTTPStatus.OK, self.server.client.get_status())
            return
        if path.startswith("/api/jobs/"):
            request_id = path[len("/api/jobs/"):]
            if not REQUEST_ID_RE.match(request_id):
                self._send_json(
                    HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid_request_id"}
                )
                return
            record = self.server.jobs.get(request_id)
            if record is None:
                self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not_found"})
                return
            self._send_json(HTTPStatus.OK, {"ok": True, "job": record})
            return
        if path == "/":
            self._send_json(
                HTTPStatus.OK,
                {
                    "ok": True,
                    "service": "agent_control_api",
                    "endpoints": [
                        "GET /api/health",
                        "GET /api/status",
                        "GET /api/jobs/<request_id>",
                        "POST /api/move",
                        "POST /api/turn",
                        "POST /api/command",
                        "POST /api/stop",
                        "POST /api/estop",
                        "POST /api/estop/clear",
                    ],
                },
            )
            return
        if path == "/favicon.ico":
            self.send_response(HTTPStatus.NO_CONTENT)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not_found"})

    # ---------------------------- POST ----------------------------
    def do_POST(self) -> None:
        if not self._authorized():
            self._reject(HTTPStatus.UNAUTHORIZED, "unauthorized")
            return
        path = urlparse(self.path).path
        if path not in (
            "/api/move",
            "/api/turn",
            "/api/command",
            "/api/stop",
            "/api/estop",
            "/api/estop/clear",
        ):
            self._reject(HTTPStatus.NOT_FOUND, "not_found")
            return

        content_type = self.headers.get("Content-Type", "")
        if "application/json" not in content_type.lower():
            self._reject(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, "content_type")
            return
        body = self._read_body()
        if body is None:
            return
        payload = self._parse_json(body)
        if payload is None:
            return

        if path == "/api/move":
            self._handle_move(payload)
        elif path == "/api/turn":
            self._handle_turn(payload)
        elif path == "/api/command":
            self._handle_command(payload)
        elif path == "/api/stop":
            ack = self.server.jobs.stop()
            self._send_ack(ack, {"ok": True})
        elif path == "/api/estop":
            ack = self.server.jobs.estop()
            if not ack.get("accepted", False):
                self._send_ack(ack, {})
                return
            _, confirmed = self._wait_driver(
                lambda driver: bool(driver.get("estop_latched")), 3.0
            )
            if not confirmed:
                self._send_json(
                    HTTPStatus.GATEWAY_TIMEOUT,
                    {
                        "ok": False,
                        "error": {
                            "code": "estop_not_confirmed",
                            "message": "driver did not confirm the emergency stop in time",
                        },
                    },
                )
                return
            self._send_json(
                HTTPStatus.OK,
                {"ok": True, "latched": True, "confirmed": True},
            )
        else:
            ack = self.server.jobs.clear_estop()
            if not ack.get("accepted", False):
                code = str(ack.get("code", "rejected"))
                status = HTTPStatus.CONFLICT if code == "not_latched" else HTTPStatus.SERVICE_UNAVAILABLE
                self._send_json(
                    status,
                    {"ok": False, "error": {"code": code, "message": ack.get("message", "")}},
                )
                return
            _, confirmed = self._wait_driver(
                lambda driver: not driver.get("estop_latched") and bool(driver.get("in_control")),
                7.0,
            )
            if not confirmed:
                self._send_json(
                    HTTPStatus.GATEWAY_TIMEOUT,
                    {
                        "ok": False,
                        "error": {
                            "code": "estop_recovery_timeout",
                            "message": "driver did not recover from the emergency stop in time",
                        },
                    },
                )
                return
            self._send_json(
                HTTPStatus.OK,
                {"ok": True, "cleared": True, "confirmed": True},
            )

    # ---------------------------- handlers ----------------------------
    def _handle_move(self, payload: dict) -> None:
        direction = payload.get("direction")
        if direction not in MOVE_DIRECTIONS:
            self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid_direction"})
            return
        distance = self._number(payload, "distance_m")
        if distance is None:
            self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid_distance_m"})
            return
        lateral = direction in ("left", "right")
        limit = 3.0 if lateral else 5.0
        if distance < 0.05 or distance > limit:
            self._send_json(
                HTTPStatus.UNPROCESSABLE_ENTITY,
                {"ok": False, "error": "distance_out_of_range"},
            )
            return
        scale = self._optional_number(payload, "max_speed_scale", 0.35)
        if scale is None or not 0.05 <= scale <= 0.5:
            self._send_json(
                HTTPStatus.UNPROCESSABLE_ENTITY,
                {"ok": False, "error": "invalid_max_speed_scale"},
            )
            return
        timeout_s = self._optional_number(payload, "timeout_s", 30.0)
        if timeout_s is None or not 1.0 <= timeout_s <= 30.0:
            self._send_json(
                HTTPStatus.UNPROCESSABLE_ENTITY,
                {"ok": False, "error": "invalid_timeout_s"},
            )
            return
        request_id = self._request_id(payload)
        if request_id is None:
            self._send_json(
                HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid_request_id"}
            )
            return

        if direction == "forward":
            forward_m, strafe_m = distance, 0.0
        elif direction == "backward":
            forward_m, strafe_m = -distance, 0.0
        elif direction == "left":
            forward_m, strafe_m = 0.0, distance
        else:
            forward_m, strafe_m = 0.0, -distance

        timeout_ms = int(timeout_s * 1000.0)
        line = (
            f"MOVE {request_id} {forward_m:.6f} {strafe_m:.6f} "
            f"{scale:.4f} {timeout_ms} 1500"
        )
        record, error = self.server.jobs.submit(
            request_id,
            "move",
            {"direction": direction, "distance_m": distance},
            line,
        )
        self._submit_response(record, error)

    def _handle_turn(self, payload: dict) -> None:
        direction = payload.get("direction")
        if direction not in TURN_DIRECTIONS:
            self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid_direction"})
            return
        angle = self._number(payload, "angle_deg")
        if angle is None:
            self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid_angle_deg"})
            return
        if angle < 1.0 or angle > 180.0:
            self._send_json(
                HTTPStatus.UNPROCESSABLE_ENTITY,
                {"ok": False, "error": "angle_out_of_range"},
            )
            return
        scale = self._optional_number(payload, "max_speed_scale", 0.35)
        if scale is None or not 0.05 <= scale <= 0.5:
            self._send_json(
                HTTPStatus.UNPROCESSABLE_ENTITY,
                {"ok": False, "error": "invalid_max_speed_scale"},
            )
            return
        timeout_s = self._optional_number(payload, "timeout_s", 30.0)
        if timeout_s is None or not 1.0 <= timeout_s <= 30.0:
            self._send_json(
                HTTPStatus.UNPROCESSABLE_ENTITY,
                {"ok": False, "error": "invalid_timeout_s"},
            )
            return
        request_id = self._request_id(payload)
        if request_id is None:
            self._send_json(
                HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid_request_id"}
            )
            return
        angle_rad = math.radians(angle)
        if direction == "right":
            angle_rad = -angle_rad
        timeout_ms = int(timeout_s * 1000.0)
        line = f"TURN {request_id} {angle_rad:.6f} {scale:.4f} {timeout_ms} 1500"
        record, error = self.server.jobs.submit(
            request_id,
            "turn",
            {"direction": direction, "angle_deg": angle},
            line,
        )
        self._submit_response(record, error)

    def _handle_command(self, payload: dict) -> None:
        command = payload.get("command")
        if not isinstance(command, str) or command not in COMMAND_CODES:
            self._send_json(
                HTTPStatus.BAD_REQUEST, {"ok": False, "error": "unknown_command"}
            )
            return
        request_id = self._request_id(payload)
        if request_id is None:
            self._send_json(
                HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid_request_id"}
            )
            return
        line = f"CMD {request_id} {COMMAND_CODES[command]}"
        record, error = self.server.jobs.submit(
            request_id, "posture", {"command": command}, line
        )
        self._submit_response(record, error)

    def _submit_response(self, record: dict | None, error: dict | None) -> None:
        if error is not None:
            code = str(error.get("code", "rejected"))
            if code in ("busy", "not_ready", "not_latched", "estop_latched"):
                status = HTTPStatus.CONFLICT
            elif code.startswith("invalid_") or code == "out_of_range":
                status = HTTPStatus.UNPROCESSABLE_ENTITY
            elif code in ("bridge_disconnected", "driver_timeout"):
                status = HTTPStatus.SERVICE_UNAVAILABLE
            else:
                status = HTTPStatus.CONFLICT
            self._send_json(status, {"ok": False, "error": error})
            return
        if record is None:
            self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "error": "missing_job"})
            return
        self._send_json(
            HTTPStatus.ACCEPTED,
            {"ok": True, "request_id": record["request_id"], "job": public_record(record)},
        )

    def _wait_driver(self, predicate, timeout: float) -> tuple[dict, bool]:
        deadline = time.monotonic() + timeout
        driver: dict = {}
        while time.monotonic() < deadline:
            status = self.server.client.get_status()
            driver = status.get("driver") or {}
            if predicate(driver):
                return driver, True
            time.sleep(0.2)
        return driver, False

    def _send_ack(self, ack: dict, success_payload: dict) -> None:
        if not ack.get("accepted", False):
            self._send_json(
                HTTPStatus.SERVICE_UNAVAILABLE,
                {
                    "ok": False,
                    "error": {
                        "code": ack.get("code", "rejected"),
                        "message": ack.get("message", ""),
                    },
                },
            )
            return
        self._send_json(HTTPStatus.ACCEPTED, success_payload)

    # ---------------------------- validation ----------------------------
    def _authorized(self) -> bool:
        token = self.server.token
        if not token:
            return True
        header = self.headers.get("Authorization", "")
        return header == "Bearer " + token

    def _request_id(self, payload: dict) -> str | None:
        request_id = payload.get("request_id")
        if request_id is None:
            return uuid.uuid4().hex
        if not isinstance(request_id, str) or not REQUEST_ID_RE.match(request_id):
            return None
        return request_id

    def _number(self, payload: dict, name: str) -> float | None:
        value = payload.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        value = float(value)
        if not math.isfinite(value):
            return None
        return value

    def _optional_number(self, payload: dict, name: str, default: float) -> float | None:
        if name not in payload:
            return default
        return self._number(payload, name)

    def _read_body(self) -> bytes | None:
        length_header = self.headers.get("Content-Length")
        if length_header is None:
            self._reject(HTTPStatus.LENGTH_REQUIRED, "length_required")
            return None
        try:
            length = int(length_header)
        except ValueError:
            self._reject(HTTPStatus.BAD_REQUEST, "invalid_length")
            return None
        if length < 0:
            self._reject(HTTPStatus.BAD_REQUEST, "invalid_length")
            return None
        if length > MAX_BODY:
            self._reject(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "body_too_large")
            return None
        return self.rfile.read(length) if length else b""

    def _parse_json(self, body: bytes) -> dict | None:
        try:
            payload = json.loads(body.decode("utf-8")) if body else {}
        except (UnicodeDecodeError, ValueError):
            self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid_json"})
            return None
        if not isinstance(payload, dict):
            self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid_payload"})
            return None
        return payload

    # ---------------------------- response ----------------------------
    def _reject(self, status: HTTPStatus, error: str) -> None:
        self.close_connection = True
        self._send_json(status, {"ok": False, "error": error}, close=True)

    def _send_json(self, status: HTTPStatus, payload: dict, close: bool = False) -> None:
        body = json.dumps(
            payload, separators=(",", ":"), ensure_ascii=False
        ).encode("utf-8")
        if close:
            self.close_connection = True
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Connection", "close" if close else "keep-alive")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            self.close_connection = True
