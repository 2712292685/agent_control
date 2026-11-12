"""Loopback TCP client for agent_control_driver.

The web/API process and the C++ driver are both local. Using TCP on
127.0.0.1 avoids WSL2 Fast DDS discovery problems while the driver remains a
normal ROS2 node.
"""

from __future__ import annotations

import json
import queue
import socket
import threading
import time
from collections.abc import Callable


class DriverClient:
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8766,
        status_timeout_sec: float = 2.0,
    ) -> None:
        self._host = host
        self._port = int(port)
        self._status_timeout = float(status_timeout_sec)
        self._lock = threading.RLock()
        self._send_lock = threading.Lock()
        self._sock: socket.socket | None = None
        self._connected = False
        self._status: dict | None = None
        self._status_rx: float | None = None
        self._status_count = 0
        self._acks: dict[str, queue.Queue[dict]] = {}
        self._results: dict[str, dict] = {}
        self._cv = threading.Condition(self._lock)
        self._stop = threading.Event()
        self.on_result: Callable[[dict], None] | None = None
        self.on_disconnect: Callable[[], None] | None = None
        self._reader = threading.Thread(target=self._run, daemon=True)
        self._heartbeat = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._reader.start()
        self._heartbeat.start()

    # ---------------------------- lifecycle ----------------------------
    def _run(self) -> None:
        while not self._stop.is_set():
            sock = None
            try:
                sock = socket.create_connection((self._host, self._port), timeout=1.0)
                sock.settimeout(0.5)
                with self._lock:
                    self._sock = sock
                    self._connected = True
                buffer = b""
                while not self._stop.is_set():
                    try:
                        data = sock.recv(8192)
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    if not data:
                        break
                    buffer += data
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        self._handle_line(line)
            except OSError:
                pass
            finally:
                with self._lock:
                    if self._sock is sock:
                        self._sock = None
                        self._connected = False
                    pending = list(self._acks.values())
                    self._acks.clear()
                for pending_queue in pending:
                    pending_queue.put(
                        {"accepted": False, "code": "bridge_disconnected"}
                    )
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
                callback = self.on_disconnect
                if callback is not None:
                    try:
                        callback()
                    except Exception:
                        pass
            self._stop.wait(1.0)

    def _heartbeat_loop(self) -> None:
        while not self._stop.is_set():
            if self._send("PING"):
                self._stop.wait(0.2)
            else:
                self._stop.wait(0.5)

    def stop(self) -> None:
        self._stop.set()
        with self._lock:
            sock = self._sock
            self._sock = None
            self._connected = False
            self._cv.notify_all()
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        for thread in (self._reader, self._heartbeat):
            thread.join(timeout=1.0)

    # ---------------------------- I/O ----------------------------
    def _send(self, line: str) -> bool:
        with self._send_lock:
            with self._lock:
                sock = self._sock
            if sock is None:
                return False
            try:
                sock.sendall((line + "\n").encode("utf-8"))
                return True
            except OSError:
                with self._lock:
                    if self._sock is sock:
                        self._sock = None
                        self._connected = False
                return False

    def _handle_line(self, raw: bytes) -> None:
        text = raw.decode("utf-8", "replace").strip()
        if not text:
            return
        if text.startswith("S:"):
            try:
                status = json.loads(text[2:])
            except ValueError:
                return
            with self._lock:
                self._status = status
                self._status_rx = time.monotonic()
                self._status_count += 1
                self._cv.notify_all()
            return
        if text.startswith("A:"):
            try:
                ack = json.loads(text[2:])
            except ValueError:
                return
            request_id = str(ack.get("request_id", ""))
            with self._lock:
                pending = self._acks.pop(request_id, None)
            if pending is not None:
                pending.put(ack)
            return
        if text.startswith("R:"):
            try:
                result = json.loads(text[2:])
            except ValueError:
                return
            request_id = str(result.get("request_id", ""))
            with self._lock:
                self._results[request_id] = result
                self._cv.notify_all()
            callback = self.on_result
            if callback is not None:
                try:
                    callback(result)
                except Exception:
                    pass
            return
        # E: protocol errors are intentionally ignored here; the driver logs them.

    def request(self, line: str, request_id: str, timeout: float = 2.0) -> dict:
        pending: queue.Queue[dict] = queue.Queue(maxsize=1)
        with self._lock:
            self._acks[request_id] = pending
        if not self._send(line):
            with self._lock:
                self._acks.pop(request_id, None)
            return {"accepted": False, "code": "bridge_disconnected"}
        try:
            return pending.get(timeout=timeout)
        except queue.Empty:
            with self._lock:
                self._acks.pop(request_id, None)
            return {"accepted": False, "code": "driver_timeout"}
        finally:
            with self._lock:
                self._acks.pop(request_id, None)

    def send_keep(self, request_id: str) -> bool:
        return self._send("KEEP " + request_id)

    def wait_result(self, request_id: str, timeout: float) -> dict | None:
        deadline = time.monotonic() + timeout
        with self._lock:
            while request_id not in self._results:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cv.wait(remaining)
            return self._results.pop(request_id)

    def peek_result(self, request_id: str) -> dict | None:
        with self._lock:
            return self._results.pop(request_id, None)

    # ---------------------------- status ----------------------------
    def get_status(self) -> dict:
        with self._lock:
            status = self._status
            rx = self._status_rx
            count = self._status_count
            connected = self._connected
        now = time.monotonic()
        age = None if rx is None else int((now - rx) * 1000)
        stale = age is None or age > int(self._status_timeout * 1000)
        return {
            "ok": True,
            "server_time_ms": int(time.time() * 1000),
            "stale": stale,
            "driver_status_age_ms": age,
            "driver": status,
            "bridge_connected": connected,
            "status_rx_count": count,
        }

    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    def status_is_fresh(self) -> bool:
        with self._lock:
            rx = self._status_rx
        return rx is not None and (time.monotonic() - rx) <= self._status_timeout
