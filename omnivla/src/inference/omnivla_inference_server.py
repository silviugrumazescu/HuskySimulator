#!/usr/bin/env python3
"""OmniVLA simulator inference server + browser dashboard.

Run INSIDE the OmniVLA container.

This process owns:
- the OmniVLA model
- active modality / language instruction
- image and pose goals
- pause / single-step state
- browser-controlled motion enable
- live dashboard
- full trajectory + observation origin, or the legacy (v, w) conversion

The ROS bridge remains the final actuator gate:
- if the server says motion_allowed=False, it publishes zero velocity
- if the server times out/errors, it publishes zero velocity
"""

from __future__ import annotations

import argparse
import gc
import base64
import io
import json
import math
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, Optional, Tuple
from urllib.parse import urlparse
from urllib.request import Request, urlopen

import numpy as np
from PIL import Image

from omnivla_predictor import MODALITY_FLAGS, OmniVLAPredictor


MODALITY_IDS = {
    "pose": 4,
    "pose_image": 5,
    "image": 6,
    "language": 7,
    "language_pose": 8,
}


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def recv_exact(conn: socket.socket, n: int) -> bytes:
    chunks = []
    remaining = n
    while remaining:
        chunk = conn.recv(remaining)
        if not chunk:
            raise ConnectionError(f"Socket closed with {remaining} bytes left")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def recv_request(conn: socket.socket) -> Tuple[Dict, bytes]:
    header_len = struct.unpack("!I", recv_exact(conn, 4))[0]
    if header_len <= 0 or header_len > 64 * 1024:
        raise ValueError(f"Invalid header length: {header_len}")

    header = json.loads(recv_exact(conn, header_len).decode("utf-8"))
    data_len = int(header.get("data_len", -1))
    if data_len < 0 or data_len > 128 * 1024 * 1024:
        raise ValueError(f"Invalid data_len: {data_len}")

    return header, recv_exact(conn, data_len)


def send_response(conn: socket.socket, response: Dict) -> None:
    payload = json.dumps(response).encode("utf-8")
    conn.sendall(struct.pack("!I", len(payload)) + payload)


def ros_raw_to_pil(header: Dict, payload: bytes) -> Image.Image:
    width = int(header["width"])
    height = int(header["height"])
    step = int(header["step"])
    encoding = str(header["encoding"]).lower()

    if width <= 0 or height <= 0 or step <= 0:
        raise ValueError(f"Invalid image geometry: width={width}, height={height}, step={step}")
    if len(payload) != height * step:
        raise ValueError(f"Expected {height * step} image bytes, got {len(payload)}")

    rows = np.frombuffer(payload, dtype=np.uint8).reshape(height, step)

    if encoding == "rgb8":
        rgb = rows[:, : width * 3].reshape(height, width, 3)
    elif encoding == "bgr8":
        bgr = rows[:, : width * 3].reshape(height, width, 3)
        rgb = bgr[:, :, ::-1]
    elif encoding == "rgba8":
        rgba = rows[:, : width * 4].reshape(height, width, 4)
        rgb = rgba[:, :, :3]
    elif encoding == "bgra8":
        bgra = rows[:, : width * 4].reshape(height, width, 4)
        rgb = bgra[:, :, [2, 1, 0]]
    elif encoding in ("mono8", "8uc1"):
        gray = rows[:, :width].reshape(height, width)
        rgb = np.repeat(gray[:, :, None], 3, axis=2)
    else:
        raise ValueError(
            f"Unsupported ROS image encoding '{encoding}'. "
            "Use rgb8/bgr8/rgba8/bgra8/mono8."
        )

    return Image.fromarray(np.ascontiguousarray(rgb), mode="RGB")


def pil_to_jpeg(image: Image.Image, quality: int = 85) -> bytes:
    buf = io.BytesIO()
    image.save(buf, format="JPEG", quality=quality)
    return buf.getvalue()


def validate_controller_settings(max_linear, max_angular, steering_gain):
    for name, value, upper in (("max_linear", max_linear, 0.5),
                               ("max_angular", max_angular, 1.0),
                               ("steering_gain", steering_gain, 20.0)):
        if not math.isfinite(value) or not 0 < value <= upper:
            raise ValueError(f"{name} must be finite, > 0 and <= {upper}")


def controller_from_trajectory(
    trajectory: np.ndarray,
    waypoint_index: int,
    waypoint_spacing: float,
    dt: float,
    max_linear: float,
    max_angular: float,
    steering_gain: float = 1.0,
) -> Tuple[float, float]:
    """Convert one local OmniVLA waypoint into a conservative (v, w)."""
    if trajectory.ndim != 2 or trajectory.shape[1] < 4:
        raise ValueError(f"Expected trajectory [N,4], got {trajectory.shape}")
    if not (0 <= waypoint_index < trajectory.shape[0]):
        raise ValueError(
            f"waypoint_index={waypoint_index} outside horizon {trajectory.shape[0]}"
        )

    chosen = trajectory[waypoint_index].astype(np.float64).copy()
    chosen[:2] *= waypoint_spacing
    dx, dy, hx, hy = chosen[:4]

    eps = 1e-8
    if abs(dx) < eps and abs(dy) < eps:
        linear = 0.0
        angular = math.atan2(float(hy), float(hx)) / dt
    elif abs(dx) < eps:
        linear = 0.0
        angular = math.copysign(math.pi / (2.0 * dt), float(dy))
    else:
        linear = float(dx) / dt
        angular = math.atan2(float(dy), float(dx)) / dt

    linear = float(np.clip(linear, 0.0, 0.5))
    angular = float(np.clip(angular, -1.0, 1.0))
    # Tune steering independently of speed. Gain > 1 intentionally tightens
    # the commanded arc; the final saturation still respects both limits.
    angular *= steering_gain

    # Saturate while approximately preserving curvature.
    if abs(linear) <= max_linear:
        if abs(angular) <= max_angular:
            return linear, angular
        if abs(angular) < eps:
            return linear, 0.0
        radius = linear / angular
        return (
            max_angular * math.copysign(abs(radius), linear),
            max_angular * math.copysign(1.0, angular),
        )

    if abs(angular) <= 0.001:
        return max_linear * math.copysign(1.0, linear), 0.0

    radius = linear / angular
    if abs(radius) >= max_linear / max_angular:
        return (
            max_linear * math.copysign(1.0, linear),
            max_linear * math.copysign(1.0, angular) / abs(radius),
        )

    return (
        max_angular * math.copysign(abs(radius), linear),
        max_angular * math.copysign(1.0, angular),
    )


class RuntimeState:
    def __init__(self, args):
        self.lock = threading.RLock()

        self.mode = args.mode
        self.instruction = args.instruction
        self.waypoint_index = args.waypoint_index
        self.waypoint_spacing = args.waypoint_spacing
        self.dt = args.dt
        self.max_linear = args.max_linear
        self.max_angular = args.max_angular
        self.steering_gain = args.steering_gain
        self.execution_mode = args.execution_mode
        self.simulator_url = args.simulator_url
        validate_controller_settings(self.max_linear, self.max_angular, self.steering_gain)
        self.stop_distance = args.stop_distance
        self.stop_yaw_deg = args.stop_yaw_deg

        self.goal_image_path = args.goal_image
        self.goal_pose_path = args.goal_pose_file

        # Browser control state.
        self.motion_enabled = False
        self.inference_paused = False
        self.step_requested = False

        # Live robot state.
        self.current_image: Optional[Image.Image] = None
        self.current_jpeg: Optional[bytes] = None
        self.current_odom: Optional[Dict[str, float]] = None
        self.frame_seq = 0
        self.last_frame_time = -math.inf

        # Goal state.
        self.goal_image: Optional[Image.Image] = None
        self.goal_jpeg: Optional[bytes] = None
        self.goal_world: Optional[Dict[str, float]] = None

        # Last prediction.
        self.ready = False
        self.message = "Waiting for camera"
        self.inference_seconds: Optional[float] = None
        self.linear_mps = 0.0
        self.angular_rps = 0.0
        self.trajectory_m = []
        self.trajectory_origin = None
        self.selected_waypoint_m = None
        self.modality_id = MODALITY_IDS[self.mode]
        self.relative_goal = None
        self.prediction_seq = 0
        self.last_prediction_time: Optional[float] = None
        self.configuration_revision = 0
        self.model_loaded = False
        self.checkpoint_loading = False
        self.checkpoint_error = ''
        self.switch_checkpoint = None
        self.model_path, self.resume_step = str(args.model_path), args.resume_step
        # Managed trials use a reset generation and simulation-time barrier.
        self.scenario_epoch = ''
        self.scenario_running = False
        self.min_sim_time = 0.0
        self.prediction_epoch = None
        self.observation_sim_time = None
        self.encoded_goal_pose = None
        self.trajectory_raw = []

        self._load_persisted_goals()

    def _load_persisted_goals(self):
        if self.goal_image_path.exists():
            try:
                self.goal_image = Image.open(self.goal_image_path).convert("RGB")
                self.goal_jpeg = pil_to_jpeg(self.goal_image)
                print(f"Loaded goal image: {self.goal_image_path}")
            except Exception as exc:
                print(f"Could not load goal image: {exc!r}")

        if self.goal_pose_path.exists():
            try:
                data = json.loads(self.goal_pose_path.read_text())
                self.goal_world = {
                    "x": float(data["x"]),
                    "y": float(data["y"]),
                    "yaw": float(data["yaw"]),
                }
                print(f"Loaded goal pose: {self.goal_world}")
            except Exception as exc:
                print(f"Could not load goal pose: {exc!r}")

    def _force_stop_locked(self, reason: str):
        self.configuration_revision += 1
        self.motion_enabled = False
        self.linear_mps = 0.0
        self.angular_rps = 0.0
        self.message = reason
        self.scenario_running = False

    def save_goal_pose(self):
        if self.goal_world is None:
            return
        self.goal_pose_path.parent.mkdir(parents=True, exist_ok=True)
        self.goal_pose_path.write_text(json.dumps(self.goal_world, indent=2))

    def set_current(self, image: Image.Image, odom):
        jpeg = pil_to_jpeg(image)
        with self.lock:
            self.current_image = image
            self.current_jpeg = jpeg
            self.last_frame_time = time.monotonic()
            if odom is not None:
                self.current_odom = odom
            self.frame_seq += 1

    def capture_goal(self, capture_image=True, capture_pose=True):
        with self.lock:
            if self.current_image is None:
                raise RuntimeError("No current camera frame is available")

            if capture_image:
                self.goal_image = self.current_image.copy()
                self.goal_jpeg = pil_to_jpeg(self.goal_image, quality=95)
                self.goal_image_path.parent.mkdir(parents=True, exist_ok=True)
                self.goal_image.save(self.goal_image_path, quality=95)

            if capture_pose:
                if self.current_odom is None:
                    raise RuntimeError("No odometry is available")
                self.goal_world = dict(self.current_odom)
                self.save_goal_pose()

            self._force_stop_locked("Goal changed; motion disabled")

    def set_relative_pose_goal(self, forward: float, left: float, yaw_deg: float):
        with self.lock:
            if self.current_odom is None:
                raise RuntimeError("No odometry is available")

            cx = self.current_odom["x"]
            cy = self.current_odom["y"]
            cyaw = self.current_odom["yaw"]

            gx = cx + math.cos(cyaw) * forward - math.sin(cyaw) * left
            gy = cy + math.sin(cyaw) * forward + math.cos(cyaw) * left
            gyaw = wrap_angle(cyaw + math.radians(yaw_deg))

            self.goal_world = {"x": gx, "y": gy, "yaw": gyaw}
            self.save_goal_pose()
            self._force_stop_locked("Pose goal changed; motion disabled")

    def clear_goals(self):
        with self.lock:
            self.goal_image = None
            self.goal_jpeg = None
            self.goal_world = None
            self.relative_goal = None
            self._force_stop_locked("Goals cleared; motion disabled")
            try:
                self.goal_image_path.unlink(missing_ok=True)
            except Exception:
                pass
            try:
                self.goal_pose_path.unlink(missing_ok=True)
            except Exception:
                pass

    def relative_pose(self, observation_odom=None):
        with self.lock:
            if self.current_odom is None or self.goal_world is None:
                return None
            c = dict(observation_odom if observation_odom is not None else self.current_odom)
            g = dict(self.goal_world)

        dx = g["x"] - c["x"]
        dy = g["y"] - c["y"]
        yaw = c["yaw"]

        # ROS base frame convention: +x forward, +y left.
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        left = -math.sin(yaw) * dx + math.cos(yaw) * dy
        rel_yaw = wrap_angle(g["yaw"] - yaw)

        return {
            "forward": forward,
            "left": left,
            "yaw_deg": math.degrees(rel_yaw),
            "distance": math.hypot(forward, left),
        }

    def motion_allowed(self):
        with self.lock:
            return bool(self.model_loaded and not self.checkpoint_loading and
                        self.motion_enabled and not self.inference_paused and self.ready
                        and time.monotonic()-self.last_frame_time < 1.0)

    def public_status(self):
        with self.lock:
            age = None
            if self.last_prediction_time is not None:
                age = max(0.0, time.time() - self.last_prediction_time)

            return {
                "mode": self.mode,
                "modality_id": self.modality_id,
                "conditioning": dict(MODALITY_FLAGS[self.mode]),
                "instruction": self.instruction,
                "ready": self.ready,
                "model_loaded": self.model_loaded,
                "checkpoint_loading": self.checkpoint_loading,
                "checkpoint_error": self.checkpoint_error,
                "configuration_revision": self.configuration_revision,
                "message": self.message,
                "motion_enabled": self.motion_enabled,
                "motion_allowed": self.motion_allowed(),
                "camera_age_seconds": None if self.current_image is None else
                    max(0.0, time.monotonic()-self.last_frame_time),
                "inference_paused": self.inference_paused,
                "step_requested": self.step_requested,
                "frame_seq": self.frame_seq,
                "prediction_seq": self.prediction_seq,
                "prediction_age_seconds": age,
                "inference_seconds": self.inference_seconds,
                "linear_mps": self.linear_mps,
                "angular_rps": self.angular_rps,
                "trajectory_m": self.trajectory_m,
                "selected_waypoint_m": self.selected_waypoint_m,
                "waypoint_index": self.waypoint_index,
                "relative_goal": self.relative_goal,
                "current_odom": self.current_odom,
                "goal_world": self.goal_world,
                "has_goal_image": self.goal_image is not None,
                "has_goal_pose": self.goal_world is not None,
                "max_linear": self.max_linear,
                "max_angular": self.max_angular,
                "steering_gain": self.steering_gain,
                "execution_mode": self.execution_mode,
                "trajectory_origin": self.trajectory_origin,
                "scenario_epoch": self.scenario_epoch,
                "scenario_running": self.scenario_running,
                "prediction_epoch": self.prediction_epoch,
                "observation_sim_time": self.observation_sim_time,
                "encoded_goal_pose": self.encoded_goal_pose,
                "trajectory_raw": self.trajectory_raw,
                "model_path": self.model_path,
                "resume_step": self.resume_step,
                "waypoint_spacing": self.waypoint_spacing,
                "waypoint_dt": self.dt,
            }


DASHBOARD_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OmniVLA Simulator Dashboard</title>
<style>
:root {
  color-scheme: dark;
  --bg:#111317; --panel:#1a1d23; --panel2:#22262e;
  --text:#f2f4f8; --muted:#aab2bf; --border:#343a46;
  --blue:#74a7ff; --green:#67d391; --yellow:#f4c56a; --red:#ff7272;
}
* { box-sizing:border-box; }
body { margin:0; background:var(--bg); color:var(--text); font-family:system-ui,-apple-system,sans-serif; }
main { max-width:1500px; margin:0 auto; padding:18px; }
h1 { margin:0 0 4px; font-size:22px; }
.sub { color:var(--muted); margin-bottom:14px; }
.topbar { display:flex; gap:8px; align-items:center; flex-wrap:wrap; margin-bottom:14px; }
.badge { border:1px solid var(--border); border-radius:999px; padding:5px 9px; background:var(--panel2); color:var(--muted); font-size:13px; }
.good { color:var(--green); }
.warn { color:var(--yellow); }
.bad { color:var(--red); }
.grid { display:grid; grid-template-columns:minmax(0,1.25fr) minmax(360px,.75fr); gap:14px; }
.card { background:var(--panel); border:1px solid var(--border); border-radius:12px; padding:14px; min-width:0; }
.card + .card { margin-top:14px; }
.media-grid { display:grid; grid-template-columns:1fr 1fr; gap:12px; }
h3 { margin:0 0 8px; font-size:15px; }
img { width:100%; aspect-ratio:4/3; object-fit:contain; background:#060708; border-radius:8px; border:1px solid var(--border); }
canvas { width:100%; aspect-ratio:1/1; display:block; background:#101319; border:1px solid var(--border); border-radius:8px; }
.metrics { display:grid; grid-template-columns:repeat(4,minmax(0,1fr)); gap:8px; margin-top:12px; }
.metric { background:var(--panel2); border-radius:8px; padding:9px; }
.metric span { color:var(--muted); font-size:12px; }
.metric b { display:block; font-size:18px; margin-top:2px; }
.control-row { display:grid; grid-template-columns:1.15fr 1fr 1fr 1fr; gap:8px; }
button, input, select, textarea {
  width:100%; font:inherit; border-radius:8px; border:1px solid var(--border);
  background:#101319; color:var(--text); padding:10px;
}
button { cursor:pointer; background:#26364f; border-color:#385176; }
button:hover { filter:brightness(1.12); }
button:disabled { opacity:.45; cursor:not-allowed; filter:none; }
button.stop { background:#6f2529; border-color:#a53a40; font-weight:800; }
button.on { background:#28563a; border-color:#3b8055; }
button.pause { background:#5b4923; border-color:#816a34; }
label { display:block; color:var(--muted); font-size:12px; margin:9px 0 5px; }
textarea { resize:vertical; min-height:66px; }
.three { display:grid; grid-template-columns:repeat(3,1fr); gap:8px; }
.two { display:grid; grid-template-columns:1fr 1fr; gap:8px; }
.info { font-family:ui-monospace,SFMono-Regular,Menlo,monospace; font-size:12px; color:var(--muted); white-space:pre-wrap; line-height:1.5; }
.frozen { color:var(--yellow); font-weight:700; }
.help { color:var(--muted); font-size:12px; line-height:1.45; margin-top:8px; }
@media (max-width:1050px) {
  .grid { grid-template-columns:1fr; }
}
@media (max-width:700px) {
  .media-grid,.control-row,.three,.two { grid-template-columns:1fr; }
  .metrics { grid-template-columns:repeat(2,1fr); }
}
</style>
</head>
<body>
<main>
  <h1>OmniVLA simulator dashboard</h1>
  <div class="sub">Live RGB, active conditioning, frozen/single-step debugging, predicted local trajectory and commanded motion.</div>

  <div class="topbar">
    <span id="connectionBadge" class="badge">connecting…</span>
    <span id="modeBadge" class="badge">mode —</span>
    <span id="inferenceBadge" class="badge">inference —</span>
    <span id="motionBadge" class="badge">motion —</span>
    <span id="messageBadge" class="badge">—</span>
  </div>

  <div class="card" style="margin-bottom:14px">
    <div class="control-row">
      <button id="stopButton" class="stop">■ STOP</button>
      <button id="motionButton">Enable motion</button>
      <button id="pauseButton">Pause inference</button>
      <button id="stepButton">Step once</button>
    </div>
    <div class="help">
      STOP immediately disables browser motion authorization. When inference is paused, the camera stays live and the last trajectory remains frozen. Step once runs exactly one new prediction while staying paused; it never authorizes motion.
    </div>
  </div>

  <div class="grid">
    <section>
      <div class="card">
        <div class="media-grid">
          <div>
            <h3>Current robot camera</h3>
            <img id="currentImage" alt="Current robot camera">
          </div>
          <div>
            <h3>Saved image goal</h3>
            <img id="goalImage" alt="Saved image goal">
          </div>
        </div>
      </div>

      <div class="card">
        <h3>Current model trajectory — robot frame <span id="frozenText"></span></h3>
        <canvas id="trajCanvas" width="700" height="700"></canvas>
        <div class="metrics">
          <div class="metric"><span>Linear command</span><b id="vMetric">—</b></div>
          <div class="metric"><span>Angular command</span><b id="wMetric">—</b></div>
          <div class="metric"><span>Inference</span><b id="inferMetric">—</b></div>
          <div class="metric"><span>Prediction</span><b id="predMetric">—</b></div>
        </div>
      </div>
    </section>

    <aside>
      <div class="card">
        <h3>Modality</h3>
        <label for="mode">Active conditioning</label>
        <select id="mode">
          <option value="image">Image goal · ID 6</option>
          <option value="pose">Pose goal · ID 4</option>
          <option value="pose_image">Pose + image · ID 5</option>
          <option value="language">Language · ID 7</option>
          <option value="language_pose">Language + pose · ID 8</option>
        </select>

        <label for="instruction">Language instruction</label>
        <textarea id="instruction" placeholder="e.g. move toward the red object"></textarea>
        <button id="applyConfig">Apply modality / instruction</button>
        <div class="help">Changing modality or instruction automatically disables motion.</div>
        <div id="conditioningInfo" class="help"></div>
      </div>

      <div class="card">
        <h3>Driving settings</h3>
        <div class="three">
          <div><label for="maxLinear">Max forward [m/s]</label><input id="maxLinear" type="number" min="0.01" max="0.5" step="0.01"></div>
          <div><label for="maxAngular">Max turn [rad/s]</label><input id="maxAngular" type="number" min="0.05" max="1" step="0.05"></div>
          <div><label for="steeringGain">Steering gain</label><input id="steeringGain" type="number" min="0.5" max="20" step="0.5"></div>
        </div>
        <button id="applyDriving" style="margin-top:8px">Apply driving settings</button>
        <div id="drivingError" class="help bad" role="alert"></div>
        <div id="executionInfo" class="help"></div>
      </div>

      <div class="card">
        <h3>Capture goal from current robot state</h3>
        <div class="two">
          <button id="captureImage">Capture image</button>
          <button id="capturePose">Capture pose</button>
        </div>
        <button id="captureBoth" style="margin-top:8px">Capture image + pose</button>
        <div class="help">Goal changes automatically disable motion.</div>
      </div>

      <div class="card">
        <h3>Set fixed pose goal relative to robot now</h3>
        <div class="three">
          <div><label>Forward [m]</label><input id="forward" type="number" step="0.1" value="2.0"></div>
          <div><label>Left [m]</label><input id="left" type="number" step="0.1" value="0.0"></div>
          <div><label>Yaw [deg]</label><input id="yaw" type="number" step="5" value="0"></div>
        </div>
        <button id="setRelative" style="margin-top:8px">Set fixed world goal</button>
      </div>

      <div class="card">
        <h3>Live state</h3>
        <div id="liveInfo" class="info">Waiting…</div>
        <button id="clearGoals" class="stop" style="margin-top:10px">Clear goals</button>
      </div>
    </aside>
  </div>
</main>

<script>
const $ = id => document.getElementById(id);
let lastFrameSeq = -1;
let firstConfigLoad = true;
let currentStatus = null;

async function api(path, options={}) {
  const r = await fetch(path, options);
  if (!r.ok) throw new Error(await r.text());
  return r.json();
}
async function post(path, body={}) {
  return api(path, {
    method:"POST",
    headers:{"Content-Type":"application/json"},
    body:JSON.stringify(body)
  });
}
function fmt(v, digits=3) {
  return (typeof v === "number" && Number.isFinite(v)) ? v.toFixed(digits) : "—";
}

function drawTrajectory(s) {
  const canvas = $("trajCanvas");
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const W = Math.max(320, Math.round(rect.width*dpr));
  const H = W;
  if (canvas.width !== W || canvas.height !== H) {
    canvas.width=W; canvas.height=H;
  }

  const ctx=canvas.getContext("2d");
  ctx.clearRect(0,0,W,H);

  const pts=Array.isArray(s.trajectory_m) ? s.trajectory_m : [];
  const goal=s.conditioning.pose_goal ? s.relative_goal : null;

  let extent=1.0;
  for (const p of pts) extent=Math.max(extent,Math.abs(p[0]||0),Math.abs(p[1]||0));
  if (goal) extent=Math.max(extent,Math.abs(goal.forward||0),Math.abs(goal.left||0));
  extent*=1.25;

  const cx=W*0.5, cy=H*0.84;
  const ppm=Math.min(W*0.42/extent,H*0.70/extent);
  const xy=(forward,left)=>[cx-left*ppm,cy-forward*ppm];

  ctx.strokeStyle="#2c323c"; ctx.lineWidth=1*dpr;
  ctx.fillStyle="#8f99a8"; ctx.font=`${11*dpr}px system-ui`;
  const step=extent>4 ? 1.0 : 0.5;

  for (let m=-extent;m<=extent;m+=step) {
    const a=xy(0,m),b=xy(extent,m);
    ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();
  }
  for (let m=0;m<=extent;m+=step) {
    const a=xy(m,-extent),b=xy(m,extent);
    ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();
  }

  if (goal) {
    const g=xy(goal.forward,goal.left);
    ctx.fillStyle="#f4c56a";
    ctx.beginPath();ctx.arc(g[0],g[1],8*dpr,0,Math.PI*2);ctx.fill();
    ctx.fillText("pose goal",g[0]+10*dpr,g[1]);
  }

  if (pts.length) {
    ctx.strokeStyle="#74a7ff"; ctx.lineWidth=3*dpr;
    ctx.beginPath();
    const r=xy(0,0);ctx.moveTo(r[0],r[1]);
    for (const p of pts) {
      const q=xy(p[0],p[1]);ctx.lineTo(q[0],q[1]);
    }
    ctx.stroke();

    pts.forEach((p,i)=>{
      const q=xy(p[0],p[1]);
      const selected=s.execution_mode!=="kinematic" && i===s.waypoint_index;
      ctx.fillStyle=selected ? "#67d391" : "#d5e2ff";
      ctx.beginPath();
      ctx.arc(q[0],q[1],(selected?7:4)*dpr,0,Math.PI*2);
      ctx.fill();
      ctx.fillStyle="#aab2bf";
      ctx.fillText(String(i),q[0]+6*dpr,q[1]-6*dpr);
    });
  }

  ctx.fillStyle="#ff7272";
  ctx.beginPath();ctx.arc(cx,cy,8*dpr,0,Math.PI*2);ctx.fill();
  ctx.strokeStyle="#ff7272";ctx.lineWidth=3*dpr;
  ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(cx,cy-28*dpr);ctx.stroke();

  ctx.fillStyle="#d7dbe2";
  ctx.fillText("robot",cx+10*dpr,cy+4*dpr);
  ctx.fillText("LEFT",12*dpr,cy);
  ctx.fillText("RIGHT",W-55*dpr,cy);
  ctx.fillText("FORWARD",cx+8*dpr,18*dpr);
}

function render(s) {
  currentStatus=s;
  const conditioning=s.conditioning;

  $("connectionBadge").textContent="CONNECTED";
  $("connectionBadge").className="badge good";
  $("modeBadge").textContent=`${s.mode} · ID ${s.modality_id}`;
  $("conditioningInfo").textContent=`Model inputs — camera: active; language: ${conditioning.lan_prompt?"active":"inactive"}; pose goal: ${conditioning.pose_goal?"active":"inactive"}; image goal: ${conditioning.image_goal?"active":"inactive"}.`;
  $("inferenceBadge").textContent=s.inference_paused ? "INFERENCE PAUSED" : "INFERENCE RUNNING";
  $("inferenceBadge").className="badge "+(s.inference_paused?"warn":"good");
  $("motionBadge").textContent=s.motion_allowed ? "MOTION ACTIVE" :
    (s.motion_enabled ? "MOTION ARMED / BLOCKED" : "MOTION OFF");
  $("motionBadge").className="badge "+(s.motion_allowed?"bad":(s.motion_enabled?"warn":"good"));
  $("messageBadge").textContent=s.message||"—";

  $("motionButton").textContent=s.motion_enabled ? "Disable motion" : "Enable motion";
  $("motionButton").className=s.motion_enabled ? "on" : "";
  $("pauseButton").textContent=s.inference_paused ? "Resume inference" : "Pause inference";
  $("pauseButton").className=s.inference_paused ? "pause" : "";
  $("stepButton").disabled=!s.inference_paused;

  const kinematic=s.execution_mode==="kinematic";
  $("vMetric").previousElementSibling.textContent=kinematic?"Path speed limit":"Linear command";
  $("wMetric").previousElementSibling.textContent=kinematic?"Turn speed limit":"Angular command";
  $("vMetric").textContent=`${kinematic?"≤ ":""}${fmt(kinematic?s.max_linear:s.linear_mps)} m/s`;
  $("wMetric").textContent=`${kinematic?"≤ ":""}${fmt(kinematic?s.max_angular:s.angular_rps)} rad/s`;
  $("steeringGain").disabled=kinematic;
  document.querySelector('label[for="maxLinear"]').textContent=kinematic?"Max path speed [m/s]":"Max forward [m/s]";
  $("executionInfo").textContent=kinematic?
    "Kinematic mode uses ideal body motion. The bridge selects waypoint-to-velocity control or full-pose playback; the simulator shows the active waypoint and measured motion. Steering gain is unused. Obstacle contacts can block motion. Applying settings disables motion.":
    "Physics mode drives the wheels. Steering gain changes turn strength; gain 1 keeps the reference conversion. Applying settings disables motion.";
  $("inferMetric").textContent=s.inference_seconds==null?"—":`${fmt(s.inference_seconds)} s`;
  $("predMetric").textContent=`#${s.prediction_seq}`;
  $("frozenText").textContent=s.inference_paused ?
    ` · FROZEN (${fmt(s.prediction_age_seconds,1)} s old)` : "";
  $("frozenText").className=s.inference_paused?"frozen":"";

  if (firstConfigLoad) {
    $("mode").value=s.mode;
    $("instruction").value=s.instruction||"";
    $("maxLinear").value=s.max_linear;
    $("maxAngular").value=s.max_angular;
    $("steeringGain").value=s.steering_gain;
    firstConfigLoad=false;
  }

  if (s.frame_seq!==lastFrameSeq) {
    $("currentImage").src=`/current.jpg?t=${Date.now()}`;
    lastFrameSeq=s.frame_seq;
  }
  if (conditioning.image_goal && s.has_goal_image) $("goalImage").src=`/goal.jpg?t=${Date.now()}`;
  else $("goalImage").removeAttribute("src");

  const od=s.current_odom, gw=s.goal_world, rg=s.relative_goal;
  $("liveInfo").textContent=
    `camera frame: ${s.frame_seq}\n`+
    `prediction: ${s.prediction_seq}\n`+
    `current odom: ${od?`x=${fmt(od.x)} y=${fmt(od.y)} yaw=${fmt(od.yaw*180/Math.PI,1)}°`:"—"}\n`+
    `pose goal input: ${!conditioning.pose_goal?"inactive":gw?`x=${fmt(gw.x)} y=${fmt(gw.y)} yaw=${fmt(gw.yaw*180/Math.PI,1)}°`:"missing"}\n`+
    `relative pose input: ${!conditioning.pose_goal?"inactive":rg?`forward=${fmt(rg.forward)} left=${fmt(rg.left)} yaw=${fmt(rg.yaw_deg,1)}° dist=${fmt(rg.distance)}m`:"—"}\n`+
    `image goal input: ${!conditioning.image_goal?"inactive":s.has_goal_image?"available":"missing"}\n`+
    `motion enabled: ${s.motion_enabled}\n`+
    `motion allowed now: ${s.motion_allowed}\n`+
    `limits: v≤${fmt(s.max_linear,2)} m/s |w|≤${fmt(s.max_angular,2)} rad/s\n`+
    `execution: ${s.execution_mode||"physics"}\n`+
    `steering gain: ${kinematic?"unused":fmt(s.steering_gain,1)}`;

  drawTrajectory(s);
}

async function refresh() {
  try {
    render(await api("/api/status"));
  } catch(e) {
    $("connectionBadge").textContent="DISCONNECTED";
    $("connectionBadge").className="badge bad";
    $("messageBadge").textContent=String(e);
  }
}

$("stopButton").addEventListener("click",async()=>{
  await post("/api/stop");
  await refresh();
});
$("motionButton").addEventListener("click",async()=>{
  const enable=!(currentStatus&&currentStatus.motion_enabled);
  await post("/api/motion",{enabled:enable});
  await refresh();
});
$("pauseButton").addEventListener("click",async()=>{
  const pause=!(currentStatus&&currentStatus.inference_paused);
  await post("/api/inference",{paused:pause});
  await refresh();
});
$("stepButton").addEventListener("click",async()=>{
  await post("/api/step");
  await refresh();
});
$("applyConfig").addEventListener("click",async()=>{
  await post("/api/config",{
    mode:$("mode").value,
    instruction:$("instruction").value
  });
  await refresh();
});
$("applyDriving").addEventListener("click",async()=>{
  $("drivingError").textContent="";
  try {
    await post("/api/controller",{
      max_linear:Number($("maxLinear").value),
      max_angular:Number($("maxAngular").value),
      steering_gain:Number($("steeringGain").value)
    });
    await refresh();
  } catch(e) {
    $("drivingError").textContent=String(e);
  }
});
$("captureImage").addEventListener("click",async()=>{
  await post("/api/capture_goal",{image:true,pose:false}); await refresh();
});
$("capturePose").addEventListener("click",async()=>{
  await post("/api/capture_goal",{image:false,pose:true}); await refresh();
});
$("captureBoth").addEventListener("click",async()=>{
  await post("/api/capture_goal",{image:true,pose:true}); await refresh();
});
$("setRelative").addEventListener("click",async()=>{
  await post("/api/set_relative_goal",{
    forward:Number($("forward").value),
    left:Number($("left").value),
    yaw_deg:Number($("yaw").value)
  });
  await refresh();
});
$("clearGoals").addEventListener("click",async()=>{
  if (!confirm("Clear saved image and pose goals?")) return;
  await post("/api/clear_goals"); await refresh();
});

setInterval(refresh,350);
refresh();
</script>
</body>
</html>
"""


def make_http_handler(state: RuntimeState):
    class Handler(BaseHTTPRequestHandler):
        server_version = "OmniVLADashboard/2.0"

        def log_message(self, fmt, *args):
            return

        def _send_json(self, obj, status=200):
            data = json.dumps(obj).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _read_json(self):
            length = int(self.headers.get("Content-Length", "0"))
            if length > 5*1024*1024:
                raise ValueError('Request exceeds 5 MiB')
            if length <= 0:
                return {}
            return json.loads(self.rfile.read(length).decode("utf-8"))

        def _scenario_action(self, action, epoch):
            # The simulator owns the command gate and optional trial recorder.
            # Never hold state.lock across this request: its worker calls us back.
            request = Request(state.simulator_url.rstrip('/')+'/api/scenarios/'+action,
                              data=json.dumps({'epoch': epoch}).encode(),
                              headers={'Content-Type': 'application/json'})
            with urlopen(request, timeout=1.5) as response:
                return json.load(response)

        def _send_jpeg(self, data: Optional[bytes]):
            if data is None:
                self.send_error(404, "No image available")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store, max-age=0")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = urlparse(self.path).path

            if path == "/":
                data = DASHBOARD_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)
                return

            if path == "/api/status":
                self._send_json(state.public_status())
                return

            if path == "/current.jpg":
                with state.lock:
                    data = state.current_jpeg
                self._send_jpeg(data)
                return

            if path == "/goal.jpg":
                with state.lock:
                    data = state.goal_jpeg
                self._send_jpeg(data)
                return

            self.send_error(404)

        def do_POST(self):
            path = urlparse(self.path).path
            try:
                body = self._read_json()

                if path == '/api/checkpoint':
                    if state.switch_checkpoint is None:
                        raise ValueError('Wait for the initial model load')
                    state.switch_checkpoint(str(body.get('checkpoint','')))
                    self._send_json({'ok': True}, status=202)
                    return
                with state.lock:
                    if state.checkpoint_loading and path != '/api/stop':
                        raise ValueError('Checkpoint is loading; motion remains disabled')

                if path.startswith('/api/scenario/'):
                    action = path.rsplit('/', 1)[-1]
                    epoch = str(body.get('epoch', ''))
                    if not epoch or len(epoch) > 80:
                        raise ValueError('A scenario reset ID is required')
                    if action == 'prepare':
                        with state.lock:
                            state._force_stop_locked('Loading scenario')
                            state.inference_paused = True
                            state.step_requested = False
                            state.scenario_epoch = epoch
                            state.min_sim_time = math.inf
                            state.prediction_epoch = None
                            state.current_image = state.current_jpeg = state.current_odom = None
                            state.ready = False
                            state.trajectory_m = []
                            state.trajectory_origin = None
                            state.trajectory_raw = []
                            state.encoded_goal_pose = None
                    elif action == 'configure':
                        mode = str(body['mode'])
                        if mode not in MODALITY_FLAGS:
                            raise ValueError('Unknown scenario modality')
                        goal = {k: float(body['goal_world'][k]) for k in ('x','y','yaw_deg')}
                        minimum = float(body['min_sim_time'])
                        linear, angular = float(body['max_linear']), float(body['max_angular'])
                        validate_controller_settings(linear, angular, 1.)
                        if not all(math.isfinite(v) for v in (*goal.values(), minimum)) or minimum < 0:
                            raise ValueError('Nonfinite scenario goal or time')
                        goal_image = None
                        if body.get('goal_jpeg_base64'):
                            raw = base64.b64decode(body['goal_jpeg_base64'], validate=True)
                            goal_image = Image.open(io.BytesIO(raw)).convert('RGB')
                        if MODALITY_FLAGS[mode]['image_goal'] and goal_image is None:
                            raise ValueError('Image mode needs a goal image')
                        instruction = str(body.get('instruction','')).strip()
                        if MODALITY_FLAGS[mode]['lan_prompt'] and not instruction:
                            raise ValueError('Language mode needs an instruction')
                        with state.lock:
                            if state.scenario_epoch != epoch:
                                raise ValueError('Scenario reset was superseded')
                            state._force_stop_locked('Scenario ready; motion disabled')
                            state.mode, state.modality_id = mode, MODALITY_IDS[mode]
                            state.instruction = instruction
                            state.goal_world = {'x': goal['x'], 'y': goal['y'], 'yaw': math.radians(goal['yaw_deg'])}
                            state.goal_image = goal_image
                            state.goal_jpeg = pil_to_jpeg(goal_image,95) if goal_image is not None else None
                            state.min_sim_time = minimum
                            state.max_linear, state.max_angular, state.steering_gain = linear, angular, 1.
                            state.ready = False
                            state.current_image = state.current_jpeg = state.current_odom = None
                    elif action == 'start':
                        with state.lock:
                            if state.scenario_epoch != epoch or state.prediction_epoch != epoch or not state.ready:
                                raise ValueError('Wait for a fresh scenario prediction')
                            if time.monotonic()-state.last_frame_time > 1:
                                raise ValueError('Scenario camera is stale')
                            state.scenario_running = True
                            state.inference_paused = False
                            state.motion_enabled = True
                            state.message = ('Scenario recording' if body.get('recording', True)
                                             else 'Model motion enabled — no recording')
                    elif action == 'stop':
                        with state.lock:
                            if state.scenario_epoch and state.scenario_epoch != epoch:
                                raise ValueError('Scenario reset was superseded')
                            state._force_stop_locked('Scenario ended; motion disabled')
                            state.inference_paused = True
                            state.step_requested = False
                    elif action == 'release':
                        with state.lock:
                            if state.scenario_epoch != epoch:
                                raise ValueError('Scenario reset was superseded')
                            state._force_stop_locked('Scenario mode released; motion disabled')
                            state.scenario_epoch = ''
                            state.prediction_epoch = None
                            state.inference_paused = True
                            state.step_requested = False
                            state.ready = False
                            state.trajectory_m = []
                    else:
                        raise ValueError('Unknown scenario action')
                    self._send_json({'ok': True})
                    return

                if path == "/api/stop":
                    with state.lock:
                        state._force_stop_locked("STOP pressed")
                        epoch = state.scenario_epoch
                    if epoch:
                        self._scenario_action('abort', epoch)
                    self._send_json({"ok": True})
                    return

                if path == "/api/motion":
                    enabled = bool(body.get("enabled", False))
                    with state.lock:
                        epoch = state.scenario_epoch
                        start_scenario = enabled and epoch and not state.scenario_running
                        if enabled and state.inference_paused and not start_scenario:
                            raise ValueError(
                                "Resume inference before enabling motion"
                            )
                        if not start_scenario:
                            state.motion_enabled = enabled
                            if not enabled:
                                state.scenario_running = False
                            state.message = "Motion enabled" if enabled else "Motion disabled"
                    if start_scenario:
                        self._scenario_action('drive', epoch)
                    elif epoch and not enabled:
                        self._scenario_action('abort', epoch)
                    self._send_json({"ok": True})
                    return

                if path == "/api/inference":
                    paused = bool(body.get("paused", False))
                    with state.lock:
                        state.inference_paused = paused
                        state.step_requested = False
                        if paused:
                            # A pause is also an actuation pause.
                            state._force_stop_locked("Inference paused; trajectory frozen")
                        else:
                            state.message = "Inference resumed"
                        epoch = state.scenario_epoch
                    if paused and epoch:
                        self._scenario_action('abort', epoch)
                    self._send_json({"ok": True})
                    return

                if path == "/api/step":
                    with state.lock:
                        if not state.inference_paused:
                            raise ValueError(
                                "Step once is available only while inference is paused"
                            )
                        state.step_requested = True
                        state.motion_enabled = False
                        state.message = "Single-step requested"
                    self._send_json({"ok": True})
                    return

                if path == "/api/config":
                    mode = str(body.get("mode", state.mode))
                    if mode not in MODALITY_FLAGS:
                        raise ValueError(
                            f"mode must be one of {list(MODALITY_FLAGS)}"
                        )
                    instruction = str(body.get("instruction", "")).strip()
                    with state.lock:
                        state.mode = mode
                        state.modality_id = MODALITY_IDS[mode]
                        state.instruction = instruction
                        state._force_stop_locked(
                            "Configuration changed; motion disabled"
                        )
                    self._send_json({"ok": True})
                    return

                if path == "/api/controller":
                    with state.lock:
                        linear = float(body.get("max_linear", state.max_linear))
                        angular = float(body.get("max_angular", state.max_angular))
                        gain = float(body.get("steering_gain", state.steering_gain))
                        validate_controller_settings(linear, angular, gain)
                        state.max_linear, state.max_angular, state.steering_gain = linear, angular, gain
                        state._force_stop_locked("Driving settings changed; motion disabled")
                    self._send_json({"ok": True})
                    return

                if path == "/api/capture_goal":
                    state.capture_goal(
                        capture_image=bool(body.get("image", True)),
                        capture_pose=bool(body.get("pose", True)),
                    )
                    self._send_json({"ok": True})
                    return

                if path == "/api/set_relative_goal":
                    forward = float(body.get("forward", 0.0))
                    left = float(body.get("left", 0.0))
                    yaw_deg = float(body.get("yaw_deg", 0.0))
                    if not all(math.isfinite(v) for v in (forward, left, yaw_deg)):
                        raise ValueError("Goal values must be finite")
                    state.set_relative_pose_goal(forward, left, yaw_deg)
                    self._send_json({"ok": True})
                    return

                if path == "/api/clear_goals":
                    state.clear_goals()
                    self._send_json({"ok": True})
                    return

                self.send_error(404)

            except Exception as exc:
                self._send_json(
                    {"ok": False, "error": repr(exc)},
                    status=400,
                )

    return Handler


def start_dashboard(state: RuntimeState, host: str, port: int):
    server = ThreadingHTTPServer((host, port), make_http_handler(state))
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"Dashboard: http://127.0.0.1:{port}")
    return server


def validate_readiness(state: RuntimeState, observation=None):
    with state.lock:
        mode = state.mode
        instruction = state.instruction
        goal_image = (
            state.goal_image.copy()
            if state.goal_image is not None
            else None
        )
        current_image = (
            state.current_image.copy()
            if state.current_image is not None
            else None
        )
        if observation is not None:
            current_image = observation[0]

    relative = state.relative_pose(observation[1] if observation is not None else None)

    if current_image is None:
        return False, "Waiting for camera", None, None, relative

    flags = MODALITY_FLAGS[mode]

    if flags["image_goal"] and goal_image is None:
        return (
            False,
            f"{mode}: capture an image goal first",
            current_image,
            None,
            relative,
        )

    if flags["pose_goal"] and relative is None:
        return (
            False,
            f"{mode}: set or capture a pose goal first",
            current_image,
            goal_image,
            None,
        )

    if flags["lan_prompt"] and not instruction:
        return (
            False,
            f"{mode}: enter a language instruction",
            current_image,
            goal_image,
            relative,
        )

    return True, "Ready", current_image, goal_image, relative


def build_response_from_state(state: RuntimeState):
    with state.lock:
        return {
            "ok": True,
            "ready": state.ready,
            "configuration_revision": state.configuration_revision,
            "prediction_seq": state.prediction_seq,
            "message": state.message,
            "mode": state.mode,
            "modality_id": state.modality_id,
            "motion_allowed": state.motion_allowed(),
            "inference_paused": state.inference_paused,
            "inference_seconds": state.inference_seconds,
            "linear_mps": state.linear_mps,
            "angular_rps": state.angular_rps,
            "relative_goal": state.relative_goal,
            "trajectory_m": state.trajectory_m,
            "trajectory_origin": state.trajectory_origin,
            "waypoint_dt": state.dt,
            "max_linear": state.max_linear,
            "max_angular": state.max_angular,
            "scenario_epoch": state.scenario_epoch,
            "prediction_epoch": state.prediction_epoch,
        }


def process_inference(state: RuntimeState, predictor: OmniVLAPredictor, observation=None):
    with state.lock:
        revision = state.configuration_revision
        epoch = state.scenario_epoch
        controller_settings = dict(waypoint_index=state.waypoint_index,
                                   waypoint_spacing=state.waypoint_spacing,
                                   dt=state.dt, max_linear=state.max_linear,
                                   max_angular=state.max_angular,
                                   steering_gain=state.steering_gain)
        paused = state.inference_paused
        step = state.step_requested
        if paused and not step:
            # Keep last trajectory visible; return zero command.
            state.linear_mps = 0.0
            state.angular_rps = 0.0
            state.message = "Inference paused; trajectory frozen"
            return build_response_from_state(state)

        if paused and step:
            # Consume exactly one requested step.
            state.step_requested = False

        # Bind the prediction to the actual input image/odometry, not a newer
        # preview that may arrive while GPU inference is running.
        ok, message, current_image, goal_image, relative = validate_readiness(state, observation)
        mode = state.mode
        instruction = state.instruction
        origin = observation[1] if observation is not None else state.current_odom
        prediction_origin = dict(origin) if origin is not None else None
        state.relative_goal = relative

    if not ok:
        with state.lock:
            state.ready = False
            state.message = message
            state.linear_mps = 0.0
            state.angular_rps = 0.0
        return build_response_from_state(state)

    flags = MODALITY_FLAGS[mode]

    if flags["pose_goal"] and relative is not None and not epoch:
        yaw_ok = abs(relative["yaw_deg"]) <= state.stop_yaw_deg
        if relative["distance"] <= state.stop_distance and yaw_ok:
            with state.lock:
                state.ready = True
                state._force_stop_locked("Pose goal reached")
                state.inference_seconds = 0.0
                state.prediction_seq += 1
                state.last_prediction_time = time.time()
            return build_response_from_state(state)

    goal_pose = None
    if flags["pose_goal"]:
        goal_pose = predictor.make_goal_pose(
            forward_m=relative["forward"],
            left_m=relative["left"],
            yaw_deg=relative["yaw_deg"],
            waypoint_spacing=state.waypoint_spacing,
        )

    t0 = time.perf_counter()
    trajectory = predictor.predict(
        current_image=current_image,
        goal_image=goal_image,
        modality=mode,
        goal_pose=goal_pose,
        language=instruction if flags["lan_prompt"] else None,
    )
    infer_s = time.perf_counter() - t0

    if trajectory.ndim != 2 or trajectory.shape[1] != 4 or not np.isfinite(trajectory).all():
        raise ValueError('Expected a finite [N,4] trajectory')
    linear, angular = ((0.0, 0.0) if state.execution_mode == 'kinematic' else
                       controller_from_trajectory(trajectory, **controller_settings))

    trajectory_m = [
        [
            float(p[0] * state.waypoint_spacing),
            float(p[1] * state.waypoint_spacing),
            float(p[2]),
            float(p[3]),
        ]
        for p in trajectory
    ]

    with state.lock:
        if state.configuration_revision != revision:
            # A prompt/goal/STOP/pause changed while GPU inference was running.
            # Keep the existing trajectory frozen until a new request uses that state.
            state.linear_mps = state.angular_rps = 0.0
            return build_response_from_state(state)
        state.ready = True
        state.modality_id = int(predictor.last_modality_id)
        state.inference_seconds = infer_s
        state.trajectory_m = trajectory_m
        state.trajectory_origin = prediction_origin
        state.prediction_epoch = epoch
        state.encoded_goal_pose = goal_pose.tolist() if goal_pose is not None else None
        state.trajectory_raw = trajectory.tolist()
        state.selected_waypoint_m = trajectory_m[state.waypoint_index]
        state.relative_goal = relative
        state.prediction_seq += 1
        state.last_prediction_time = time.time()

        if state.inference_paused:
            # Single-step while paused: update visualization but never actuate.
            state.linear_mps = 0.0
            state.angular_rps = 0.0
            state.message = "Single-step complete; trajectory frozen"
        else:
            state.linear_mps = linear
            state.angular_rps = angular
            state.message = "Running"

    return build_response_from_state(state)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=5555)
    p.add_argument("--dashboard-host", default="0.0.0.0")
    p.add_argument("--dashboard-port", type=int, default=8080)
    p.add_argument("--simulator-url", default="http://127.0.0.1:8211",
                   help="Isaac controls used for manual motion in a loaded scenario.")

    p.add_argument("--model-path", default="./omnivla-original")
    p.add_argument("--resume-step", type=int, default=120000)
    p.add_argument("--mode", choices=list(MODALITY_FLAGS), default="image")
    p.add_argument("--instruction", default="")

    p.add_argument("--goal-image", type=Path, default=Path("/output/sim_goal.jpg"))
    p.add_argument(
        "--goal-pose-file",
        type=Path,
        default=Path("/output/sim_goal_pose.json"),
    )

    p.add_argument("--waypoint-index", type=int, default=4)
    p.add_argument("--waypoint-spacing", type=float, default=0.1)
    p.add_argument("--dt", type=float, default=1.0 / 3.0)
    p.add_argument("--max-linear", type=float, default=0.10)
    p.add_argument("--max-angular", type=float, default=0.25)
    p.add_argument("--steering-gain", type=float, default=1.0,
                   help="Multiply steering before velocity limits; 1 keeps the original arc.")
    p.add_argument("--execution-mode", choices=("physics", "kinematic"), default="physics")
    p.add_argument("--stop-distance", type=float, default=0.25)
    p.add_argument("--stop-yaw-deg", type=float, default=20.0)
    p.add_argument("--socket-timeout", type=float, default=30.0)
    args = p.parse_args()

    state = RuntimeState(args)
    dashboard = start_dashboard(
        state,
        host=args.dashboard_host,
        port=args.dashboard_port,
    )

    print("Loading OmniVLA once...")
    predictor = OmniVLAPredictor(
        model_path=args.model_path,
        resume_step=args.resume_step,
    )
    with state.lock:
        state.model_loaded = True

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.host, args.port))
    sock.listen(8)
    print(f"Inference socket: {args.host}:{args.port}")
    inference_lock = threading.Lock()

    def switch_checkpoint(name):
        # Use the authors' normal loader, with only one GPU model resident.
        choices = {'original':('omnivla-original',120000), 'cast':('omnivla-finetuned-cast',210000)}
        if name not in choices:
            raise ValueError('Checkpoint must be original or cast')
        folder, step = choices[name]
        target = Path(args.model_path).parent/folder
        if not (target/'config.json').is_file():
            raise ValueError('Checkpoint is not installed: '+str(target))
        with state.lock:
            if state.checkpoint_loading or state.scenario_running or state.motion_enabled:
                raise ValueError('Finish the trial and stop motion before switching checkpoints')
            state._force_stop_locked('Loading checkpoint; motion disabled')
            state.inference_paused = True
            state.step_requested = False
            state.model_loaded = False
            state.checkpoint_loading, state.checkpoint_error = True, ''
            state.ready = False
            state.prediction_epoch = None
            state.trajectory_m, state.trajectory_raw = [], []
            state.scenario_epoch = ''

        def load():
            nonlocal predictor
            try:
                with inference_lock:
                    predictor = None
                    gc.collect()
                    import torch
                    torch.cuda.empty_cache()
                    predictor = OmniVLAPredictor(model_path=str(target), resume_step=step)
                with state.lock:
                    state.model_path, state.resume_step = str(target), step
                    state.model_loaded = True
                    state.message = 'Checkpoint loaded; Load / Reset a scenario'
            except Exception as exc:
                with state.lock:
                    state.checkpoint_error = str(exc)
                    state.message = 'Checkpoint load failed: '+str(exc)
            finally:
                with state.lock:
                    state.checkpoint_loading = False
        threading.Thread(target=load, name='checkpoint-loader', daemon=True).start()

    state.switch_checkpoint = switch_checkpoint

    def handle_request(conn, addr):
        with conn:
            conn.settimeout(args.socket_timeout)
            try:
                header, payload = recv_request(conn)
                op = str(header.get("op", "infer"))
                image = ros_raw_to_pil(header, payload)
                odom = header.get("odom")
                if odom is not None:
                    odom = {key: float(odom[key]) for key in ("x", "y", "yaw")}
                def observation_is_current():
                    if not state.scenario_epoch:
                        return not header.get('scenario_epoch')
                    stamp, odom_stamp = header.get('sim_time'), header.get('odom_sim_time')
                    return (header.get('scenario_epoch') == state.scenario_epoch and
                            isinstance(stamp, (int,float)) and math.isfinite(stamp) and
                            isinstance(odom_stamp, (int,float)) and math.isfinite(odom_stamp) and
                            stamp >= state.min_sim_time and odom_stamp >= state.min_sim_time)
                if op == "preview":
                    with state.lock:
                        if observation_is_current():
                            state.set_current(image=image, odom=odom)
                    send_response(conn, build_response_from_state(state))
                    return
                if op != "infer":
                    raise ValueError(f"Unknown operation: {op}")
                # Only one model call at a time; previews stay independent.
                with inference_lock:
                    with state.lock:
                        if not state.model_loaded or state.checkpoint_loading:
                            send_response(conn, dict(build_response_from_state(state), motion_allowed=False,
                                                     ready=False, message='Checkpoint unavailable or loading'))
                            return
                        if not observation_is_current():
                            send_response(conn, dict(build_response_from_state(state), motion_allowed=False,
                                                     message='Waiting for post-reset camera and odometry'))
                            return
                        state.set_current(image=image, odom=odom)
                        state.observation_sim_time = header.get('sim_time')
                    response = process_inference(state, predictor, observation=(image, odom))
                send_response(conn, response)
                if response.get("ready") and (not response.get("inference_paused") or
                                               response.get("message") == "Single-step complete; trajectory frozen"):
                    print(f"{response['mode']} motion={response['motion_allowed']} "
                          f"v={response['linear_mps']:+.3f} w={response['angular_rps']:+.3f} "
                          f"infer={response['inference_seconds']}")
            except Exception as exc:
                try:
                    send_response(conn, {"ok": False, "motion_allowed": False, "error": repr(exc)})
                except Exception:
                    pass
                print(f"Request from {addr} failed: {exc!r}")

    try:
        while True:
            conn, addr = sock.accept()
            threading.Thread(target=handle_request, args=(conn, addr), daemon=True).start()
    except KeyboardInterrupt:
        print("\nStopping server.")
    finally:
        with state.lock:
            state._force_stop_locked("Server stopping")
        sock.close()
        dashboard.shutdown()


if __name__ == "__main__":
    main()
