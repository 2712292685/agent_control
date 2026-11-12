"""rclpy lifecycle wrapper around the HTTP API and the driver TCP client."""

from __future__ import annotations

import ipaddress
import os
import signal
import threading

import rclpy
from rclpy.node import Node

from .driver_client import DriverClient
from .http_server import ControlHTTPServer, ControlRequestHandler
from .job_manager import JobManager
from .urls import list_ipv4


def _is_loopback(address: str) -> bool:
    try:
        return ipaddress.ip_address(address).is_loopback
    except ValueError:
        return address in ("localhost", "")


class AgentControlApiNode(Node):
    def __init__(self) -> None:
        super().__init__("agent_control_api")
        self.port = int(self.declare_parameter("port", 8100).value)
        self.bind_address = str(
            self.declare_parameter("bind_address", "127.0.0.1").value
        )
        self.bridge_host = str(
            self.declare_parameter("bridge_host", "127.0.0.1").value
        )
        self.bridge_port = int(self.declare_parameter("bridge_port", 8766).value)
        self.status_timeout_sec = float(
            self.declare_parameter("status_timeout_sec", 2.0).value
        )
        self.job_timeout_sec = float(
            self.declare_parameter("job_timeout_sec", 45.0).value
        )
        self.poll_lease_timeout_sec = float(
            self.declare_parameter("poll_lease_timeout_sec", 1.5).value
        )
        self.token = os.environ.get("AGENT_CONTROL_TOKEN", "").strip()

        if not _is_loopback(self.bind_address) and not self.token:
            raise RuntimeError(
                "bind_address is not loopback; set AGENT_CONTROL_TOKEN before "
                "exposing the API"
            )

        self.client = DriverClient(
            host=self.bridge_host,
            port=self.bridge_port,
            status_timeout_sec=self.status_timeout_sec,
        )
        self.jobs = JobManager(
            self.client,
            poll_lease_timeout_sec=self.poll_lease_timeout_sec,
            job_timeout_sec=self.job_timeout_sec,
        )
        self.server = ControlHTTPServer(
            (self.bind_address, self.port),
            ControlRequestHandler,
            self.jobs,
            self.client,
            token=self.token,
        )
        self.server_thread = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.server_thread.start()
        self._print_urls()
        if self.token:
            self.get_logger().info("API token authentication is enabled")

    def _print_urls(self) -> None:
        if _is_loopback(self.bind_address):
            self.get_logger().info(
                "Windows 本机访问: http://127.0.0.1:%d/" % self.port
            )
        else:
            self.get_logger().info(
                "API listening on %s:%d" % (self.bind_address, self.port)
            )
        for address in list_ipv4():
            self.get_logger().info(
                "其他设备访问: http://%s:%d/" % (address, self.port)
            )

    def close(self) -> None:
        try:
            self.server.shutdown()
            self.server.server_close()
        except Exception:
            pass
        self.jobs.shutdown()
        self.client.stop()


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node: AgentControlApiNode | None = None
    try:
        node = AgentControlApiNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as exception:
        rclpy.logging.get_logger("agent_control_api").fatal(
            "API 节点启动失败: %s" % exception
        )
        return 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda _signum, _frame: rclpy.try_shutdown())
    raise SystemExit(main())
