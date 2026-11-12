"""Async motion/posture job records, idempotency and the client lease.

The HTTP layer returns 202 immediately and the Windows tool polls the job.
Only while the tool is polling does the API send KEEP to the driver. If the
Windows process dies or the network partitions, KEEP stops and the driver's
motion lease expires, so the robot stops instead of continuing unattended.
"""

from __future__ import annotations

import copy
import threading
import time
import uuid

from .driver_client import DriverClient

TERMINAL_STATES = {"succeeded", "failed", "aborted", "timed_out"}


def public_record(record: dict) -> dict:
    return {key: value for key, value in record.items() if not key.startswith("_")}


class JobManager:
    def __init__(
        self,
        client: DriverClient,
        poll_lease_timeout_sec: float = 1.5,
        job_timeout_sec: float = 45.0,
    ) -> None:
        self._client = client
        self._poll_lease_timeout = float(poll_lease_timeout_sec)
        self._job_timeout = float(job_timeout_sec)
        self._lock = threading.RLock()
        self._jobs: dict[str, dict] = {}
        self._last_poll: dict[str, float] = {}
        self._active_id: str | None = None
        self._stop = threading.Event()
        self._lease_thread = threading.Thread(target=self._lease_loop, daemon=True)
        client.on_result = self._on_result
        client.on_disconnect = self._on_disconnect
        self._lease_thread.start()

    # ---------------------------- lifecycle ----------------------------
    def shutdown(self) -> None:
        self._stop.set()
        self._lease_thread.join(timeout=1.0)

    def _lease_loop(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                active_id = self._active_id
                record = self._jobs.get(active_id) if active_id else None
                last_poll = self._last_poll.get(active_id, 0.0) if active_id else 0.0
                created = record.get("_created_monotonic", 0.0) if record else 0.0
                running = record is not None and record.get("state") == "running"
            if active_id and running:
                now = time.monotonic()
                if now - last_poll <= self._poll_lease_timeout:
                    self._client.send_keep(active_id)
                if now - created > self._job_timeout:
                    with self._lock:
                        record = self._jobs.get(active_id)
                        if record is not None and record.get("state") == "running":
                            record["state"] = "timed_out"
                            record["ok"] = False
                            record["error"] = {
                                "code": "timeout",
                                "message": "job did not finish before the API timeout",
                            }
                            record["updated_ms"] = int(time.time() * 1000)
                            self._active_id = None
            self._stop.wait(0.25)

    # ---------------------------- submit / query ----------------------------
    def submit(
        self,
        request_id: str,
        kind: str,
        requested: dict,
        command_line: str,
        ack_timeout: float = 2.0,
    ) -> tuple[dict | None, dict | None]:
        with self._lock:
            existing = self._jobs.get(request_id)
            if existing is not None:
                return copy.deepcopy(public_record(existing)), None
            if self._active_id is not None:
                active = self._jobs.get(self._active_id)
                if active is not None and active.get("state") not in TERMINAL_STATES:
                    return None, {
                        "code": "busy",
                        "message": "another job is active; stop it first",
                    }
            now_ms = int(time.time() * 1000)
            record = {
                "ok": True,
                "request_id": request_id,
                "kind": kind,
                "state": "running",
                "requested": requested,
                "result": None,
                "error": None,
                "created_ms": now_ms,
                "updated_ms": now_ms,
                "_created_monotonic": time.monotonic(),
            }
            self._jobs[request_id] = record
            self._last_poll[request_id] = time.monotonic()
            self._active_id = request_id

        ack = self._client.request(command_line, request_id, timeout=ack_timeout)
        if not ack.get("accepted", False):
            code = str(ack.get("code", "rejected"))
            message = str(ack.get("message", "driver rejected the command"))
            with self._lock:
                record = self._jobs.get(request_id)
                if record is not None and record.get("state") not in TERMINAL_STATES:
                    record["ok"] = False
                    record["state"] = "failed"
                    record["error"] = {"code": code, "message": message}
                    record["updated_ms"] = int(time.time() * 1000)
                if self._active_id == request_id:
                    self._active_id = None
            return copy.deepcopy(public_record(record)), {"code": code, "message": message}

        with self._lock:
            return copy.deepcopy(public_record(self._jobs.get(request_id))), None

    def get(self, request_id: str) -> dict | None:
        with self._lock:
            record = self._jobs.get(request_id)
            if record is None:
                return None
            self._last_poll[request_id] = time.monotonic()
            return copy.deepcopy(public_record(record))

    def active_id(self) -> str | None:
        with self._lock:
            return self._active_id

    # ---------------------------- commands ----------------------------
    def stop(self) -> dict:
        request_id = uuid.uuid4().hex
        return self._client.request(f"STOP {request_id}", request_id)

    def estop(self) -> dict:
        request_id = uuid.uuid4().hex
        return self._client.request(f"ESTOP {request_id}", request_id)

    def clear_estop(self) -> dict:
        request_id = uuid.uuid4().hex
        return self._client.request(f"ECLEAR {request_id}", request_id)

    # ---------------------------- driver callbacks ----------------------------
    def _on_result(self, result: dict) -> None:
        request_id = str(result.get("request_id", ""))
        if not request_id:
            return
        state = str(result.get("state", "failed"))
        with self._lock:
            record = self._jobs.get(request_id)
            if record is None:
                record = {
                    "ok": False,
                    "request_id": request_id,
                    "kind": str(result.get("kind", "unknown")),
                    "state": state,
                    "requested": {},
                    "result": result.get("result"),
                    "error": result.get("error"),
                    "created_ms": int(time.time() * 1000),
                    "updated_ms": int(time.time() * 1000),
                    "_created_monotonic": time.monotonic(),
                }
                self._jobs[request_id] = record
            record["state"] = state
            record["ok"] = state == "succeeded"
            record["result"] = result.get("result")
            record["error"] = result.get("error")
            record["updated_ms"] = int(time.time() * 1000)
            if state in TERMINAL_STATES and self._active_id == request_id:
                self._active_id = None

    def _on_disconnect(self) -> None:
        with self._lock:
            active_id = self._active_id
            if active_id is None:
                return
            record = self._jobs.get(active_id)
            if record is not None and record.get("state") not in TERMINAL_STATES:
                record["ok"] = False
                record["state"] = "aborted"
                record["error"] = {
                    "code": "driver_disconnected",
                    "message": "driver TCP link was lost",
                }
                record["updated_ms"] = int(time.time() * 1000)
            self._active_id = None
