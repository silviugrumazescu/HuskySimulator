#!/usr/bin/env python3
"""ROS 2 transport + fail-safe actuator bridge for OmniVLA.

Run in a ROS 2 Jazzy container; use simulation time for Gazebo or Isaac.

The browser/server owns the user-facing motion and inference controls.
This bridge is the final actuator gate:

- motion_allowed=False -> publish zero TwistStamped
- server exception/timeout -> publish zero TwistStamped
- authorized prediction -> wheel-command transport or ideal trajectory execution

Kinematic mode sends the raw waypoints to the simulator's existing local HTTP
control server, selecting pose playback or the authors' waypoint conversion.
Physics mode keeps the original ROS velocity commands. It also sends
camera previews faster than model inference so the browser feed
remains live while the model is paused or running.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
import socket
import struct
import time
from typing import Dict, Optional
from urllib.request import Request, urlopen

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def recv_exact(sock: socket.socket, n: int) -> bytes:
    chunks = []
    remaining = n
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError(f"Socket closed with {remaining} bytes remaining")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def send_request(
    host: str,
    port: int,
    image_msg: Image,
    odom: Optional[Dict[str, float]],
    op: str,
    timeout: float,
    scenario_epoch: str = '',
    odom_sim_time=None,
) -> Dict:
    payload = bytes(image_msg.data)
    header = {
        "op": op,
        "width": int(image_msg.width),
        "height": int(image_msg.height),
        "encoding": str(image_msg.encoding),
        "step": int(image_msg.step),
        "data_len": len(payload),
        "frame_id": str(image_msg.header.frame_id),
        "odom": odom,
        "sim_time": image_msg.header.stamp.sec+image_msg.header.stamp.nanosec*1e-9,
        "odom_sim_time": odom_sim_time,
        "scenario_epoch": scenario_epoch,
    }
    h = json.dumps(header).encode("utf-8")

    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(struct.pack("!I", len(h)))
        sock.sendall(h)
        sock.sendall(payload)

        response_len = struct.unpack("!I", recv_exact(sock, 4))[0]
        if response_len <= 0 or response_len > 32 * 1024 * 1024:
            raise ValueError(f"Invalid response length: {response_len}")
        return json.loads(recv_exact(sock, response_len).decode("utf-8"))


def send_trajectory(url, command):
    """Use the simulator's existing local control server; no extra ROS messages."""
    request = Request(url.rstrip('/')+'/api/trajectory', data=json.dumps(command).encode(),
                      headers={'Content-Type': 'application/json'}, method='POST')
    with urlopen(request, timeout=0.2) as response:
        return json.load(response)


class OmniVLABridge(Node):
    def __init__(self, args):
        super().__init__("omnivla_ros_bridge")
        self.args = args
        self.latest_image: Optional[Image] = None
        self.latest_odom: Optional[Dict[str, float]] = None
        self.image_odom = None
        self.odom_sim_time = self.image_odom_sim_time = None
        self.odom_epoch = self.image_epoch = ''
        self.odom_received = -math.inf
        self.frame_counter = 0
        self.last_infer_frame = -1
        self.last_preview_frame = -1
        self.last_wait_message = None
        self.last_logged_prediction = -1
        self.image_received = -math.inf
        self.workers = ThreadPoolExecutor(max_workers=3)
        self.jobs = {}
        self.command = None
        self.dashboard_status = None
        self.status_received = -math.inf
        self.was_actuating = False
        self.stop_published = False
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self.create_subscription(
            Image,
            args.camera_topic,
            self.on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            args.odom_topic,
            self.on_odom,
            qos_profile_sensor_data,
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            args.cmd_topic,
            10,
        )

        self.infer_timer = self.create_timer(
            1.0 / args.rate,
            self.infer_step,
            clock=self.wall_clock,
        )
        self.preview_timer = self.create_timer(
            1.0 / args.preview_rate,
            self.preview_step,
            clock=self.wall_clock,
        )

        self.create_timer(0.1, self.status_step, clock=self.wall_clock)
        self.create_timer(1.0 / args.command_rate, self.control_step, clock=self.wall_clock)

        self.get_logger().info(f"camera: {args.camera_topic}")
        self.get_logger().info(f"odom:   {args.odom_topic}")
        self.get_logger().info(f"cmd:    {args.cmd_topic} [TwistStamped]")
        self.get_logger().info(
            f"server: {args.server_host}:{args.server_port}"
        )
        self.get_logger().info(
            f"inference={args.rate:.2f} Hz preview={args.preview_rate:.2f} Hz"
        )
        if args.execution_mode == 'kinematic':
            self.get_logger().info(f'ideal body controller={args.kinematic_controller}; '
                                   f'waypoint index={args.waypoint_index} (zero based)')
        self.get_logger().info(
            "Motion authorization is controlled by the browser dashboard."
        )

    def on_image(self, msg: Image):
        self.latest_image = msg
        self.image_epoch = msg.header.frame_id.partition('__scenario_')[2]
        paired = self.latest_odom is not None and self.odom_epoch == self.image_epoch
        self.image_odom = dict(self.latest_odom) if paired else None
        self.image_odom_sim_time = self.odom_sim_time if paired else None
        self.image_received = time.monotonic()
        self.frame_counter += 1

    def on_odom(self, msg: Odometry):
        self.odom_sim_time = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.odom_epoch = msg.header.frame_id.partition('__scenario_')[2]
        q = msg.pose.pose.orientation
        self.latest_odom = {
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "yaw": yaw_from_quaternion(q.x, q.y, q.z, q.w),
        }
        self.odom_received = time.monotonic()

    def make_cmd(self, linear: float, angular: float) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        return msg

    def publish_stop(self):
        self.cmd_pub.publish(self.make_cmd(0.0, 0.0))
        if self.args.execution_mode == 'kinematic':
            send_trajectory(self.args.simulator_url, {'enabled': False,
                'scenario_epoch': (self.dashboard_status or {}).get('scenario_epoch', '')})

    def start_request(self, op):
        if op in self.jobs or self.latest_image is None:
            return
        self.jobs[op] = (self.workers.submit(
            send_request, self.args.server_host, self.args.server_port,
            self.latest_image, self.image_odom, op, self.args.timeout,
            self.image_epoch, self.image_odom_sim_time,
        ), time.monotonic())

    def preview_step(self):
        if self.frame_counter != self.last_preview_frame and "preview" not in self.jobs:
            self.last_preview_frame = self.frame_counter
            self.start_request("preview")

    def infer_step(self):
        if self.frame_counter != self.last_infer_frame and "infer" not in self.jobs:
            self.last_infer_frame = self.frame_counter
            self.start_request("infer")

    def status_step(self):
        if "status" not in self.jobs:
            def read_status():
                with urlopen(self.args.dashboard_url.rstrip("/")+"/api/status", timeout=0.3) as response:
                    return json.load(response)
            self.jobs["status"] = (self.workers.submit(read_status), time.monotonic())

    def control_step(self):
        """ROS stays responsive while workers do inference, previews and dashboard I/O."""
        now = time.monotonic()
        for op, (future, started) in list(self.jobs.items()):
            if not future.done():
                continue
            del self.jobs[op]
            try:
                response = future.result()
                if op == 'actuate':
                    if not response.get('ok'):
                        raise RuntimeError(response.get('error', 'Trajectory command failed'))
                elif op == "status":
                    self.dashboard_status = response
                    self.status_received = now
                elif not response.get("ok", False):
                    raise RuntimeError(response.get("error", "Server request failed"))
                elif op == "preview" and not response.get("motion_allowed", False):
                    self.command = None
                elif op == "infer":
                    if self.args.execution_mode == 'kinematic' and response.get('ready'):
                        if not all(key in response for key in ('trajectory_m','trajectory_origin',
                                                              'waypoint_dt','max_linear','max_angular')):
                            raise ValueError('Inference server does not provide trajectory execution data')
                    v, w = float(response.get("linear_mps", 0)), float(response.get("angular_rps", 0))
                    if not all(math.isfinite(value) for value in (v, w)):
                        raise ValueError("Inference returned non-finite velocities")
                    self.command = dict(response, linear_mps=v, angular_rps=w, started=started)
                    if response.get("ready") and response.get("prediction_seq") != self.last_logged_prediction:
                        self.last_logged_prediction = response.get("prediction_seq")
                        self.get_logger().info(
                            f"{response.get('mode')}: infer={response.get('inference_seconds')}s "
                            f"motion={response.get('motion_allowed')} execution={self.args.execution_mode} "
                            f"v={v:+.3f} w={w:+.3f}"
                        )
                    elif not response.get("ready") and response.get("message") != self.last_wait_message:
                        self.last_wait_message = response.get("message")
                        self.get_logger().info(self.last_wait_message or "Waiting for server")
            except Exception as exc:
                self.command = None
                if op == "status":
                    self.dashboard_status = None
                else:
                    self.get_logger().error(f"OmniVLA {op} failed: {exc!r}")

        status, command = self.dashboard_status, self.command
        allowed = bool(
            status and command and status.get("motion_allowed") and command.get("motion_allowed")
            and status.get("configuration_revision", 0) == command.get("configuration_revision", 0)
            and status.get('execution_mode', 'physics') == self.args.execution_mode
            and status.get('scenario_epoch', '') == command.get('scenario_epoch', '')
            and (not status.get('scenario_epoch') or
                 command.get('prediction_epoch') == status.get('scenario_epoch'))
            and now-self.status_received < 0.5
            and now-self.image_received < 1.0
            and now-command["started"] < self.args.command_timeout
            and (self.args.execution_mode != 'kinematic' or
                 (now-self.odom_received < 0.5 and command.get('trajectory_origin') is not None
                  and bool(command.get('trajectory_m'))))
        )
        if self.args.execution_mode == 'kinematic':
            # One outstanding HTTP command preserves ordering through STOP.
            # Repeat the same plan ID as a heartbeat, without restarting playback.
            if 'actuate' not in self.jobs and (allowed or self.was_actuating or not self.stop_published):
                payload = {'enabled': False, 'scenario_epoch': (status or {}).get('scenario_epoch', '')}
                if allowed:
                    payload = {key: command[key] for key in ('trajectory_m', 'trajectory_origin',
                               'waypoint_dt', 'max_linear', 'max_angular')}
                    payload.update(enabled=True, expires_at=now+0.25,
                                   scenario_epoch=command.get('scenario_epoch', ''),
                                   plan_id=f"{command['started']:.9f}:{command['prediction_seq']}")
                    if self.args.kinematic_controller == 'waypoint':
                        payload['waypoint_index'] = self.args.waypoint_index
                self.jobs['actuate'] = (self.workers.submit(send_trajectory, self.args.simulator_url,
                                                          payload), now)
                self.was_actuating, self.stop_published = allowed, not allowed
            return
        if allowed:
            self.cmd_pub.publish(self.make_cmd(command["linear_mps"], command["angular_rps"]))
            self.was_actuating = True
            self.stop_published = False
        elif self.was_actuating or not self.stop_published:
            # One explicit stop, then silence: idle autonomy must not fight manual driving.
            self.publish_stop()
            self.was_actuating = False
            self.stop_published = True

    def destroy_node(self):
        try:
            if 'actuate' in self.jobs:
                try:
                    self.jobs['actuate'][0].result(timeout=0.3)
                except Exception:
                    pass
            self.publish_stop()
        except Exception:
            pass
        self.workers.shutdown(wait=False, cancel_futures=True)
        super().destroy_node()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--camera-topic", required=True)
    p.add_argument("--odom-topic", required=True)
    p.add_argument("--cmd-topic", required=True)
    p.add_argument("--server-host", default="127.0.0.1")
    p.add_argument("--server-port", type=int, default=5555)
    p.add_argument("--rate", type=float, default=1.0)
    p.add_argument("--preview-rate", type=float, default=5.0)
    p.add_argument("--timeout", type=float, default=30.0)
    p.add_argument("--dashboard-url", default="http://127.0.0.1:8080")
    p.add_argument("--command-rate", type=float, default=20.0)
    p.add_argument("--command-timeout", type=float, default=2.0)
    p.add_argument('--execution-mode', choices=('physics', 'kinematic'), default='physics')
    p.add_argument('--kinematic-controller', choices=('trajectory', 'waypoint'), default='waypoint',
                   help='Ideal pose playback or the authors\' selected-waypoint velocity conversion.')
    p.add_argument('--waypoint-index', type=int, default=4,
                   help='Zero-based point used by the kinematic waypoint controller; 4 is point 5.')
    p.add_argument('--simulator-url', default='http://127.0.0.1:8211')
    args, ros_args = p.parse_known_args()

    if not math.isfinite(args.rate) or args.rate <= 0 or args.rate > 10:
        p.error("--rate must be > 0 and <= 10 Hz")
    if not math.isfinite(args.preview_rate) or args.preview_rate <= 0 or args.preview_rate > 15:
        p.error("--preview-rate must be > 0 and <= 15 Hz")

    if not math.isfinite(args.command_rate) or not 0 < args.command_rate <= 100:
        p.error("--command-rate must be finite and within (0, 100] Hz")
    if not math.isfinite(args.command_timeout) or args.command_timeout <= 0:
        p.error("--command-timeout must be finite and positive")
    if not 0 <= args.waypoint_index < 8:
        p.error('--waypoint-index must be within the eight-point horizon (0 through 7)')
    rclpy.init(args=ros_args)
    node = OmniVLABridge(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
