# Husky A200 in Isaac Sim

See the [root setup guide](../README.md) for prerequisites, image builds and
checkpoint downloads. This directory provides the full-warehouse simulator,
native Isaac viewer in the browser, manual driving, OmniVLA integration and
editable evaluation scenarios.

## Start and stop

From the project root, after building and downloading a checkpoint:

```bash
export USER_UID="$(id -u)" USER_GID="$(id -g)"
docker compose -f isaac_sim/compose.yaml --profile omnivla up -d live viewer omnivla bridge
```

Open **http://127.0.0.1:8211/** for the simulator/scenario panel and
**http://127.0.0.1:8080/** for OmniVLA's camera, prompts and predictions.
The model loads after Isaac is healthy. Motion starts disabled.

For manual simulation without the model:

```bash
docker compose -f isaac_sim/compose.yaml --profile interactive up -d live viewer
```

Stop only this workflow, keeping containers and data:

```bash
docker compose -f isaac_sim/compose.yaml --profile omnivla stop bridge omnivla viewer live
```

## Live viewing and driving

The page wraps NVIDIA's native editor stream with manual and scenario controls.
Enable manual driving, then hold the arrow buttons or use WASD with the driving
panel focused. Release a key/button to brake; Space or STOP disables motion.
Disable model motion before manual driving. Clicking into the native viewer
returns focus to its camera/editor controls.

The WebRTC viewer uses HTTP 8210, signaling TCP 49100 and media UDP 47998.
Only one streaming client can connect at a time. A browser in a remote desktop
can open the localhost URLs above; ordinary HTTP forwarding through SSH or
VS Code does not forward WebRTC media.

If the browser reports "No ICE candidate pairs were nominated", close other
viewer tabs and inspect the advertised stream address in
`http://127.0.0.1:8211/api/state`. The default is the workstation's default-route
IPv4 address. Override it with `ISAAC_STREAM_HOST=<workstation-IPv4>` on the
startup command if needed; it must be reachable from the browser.

## Scenarios and manual ablation study

1. Enter a study name and repeat number.
2. Choose a loaded checkpoint, goal modality and camera preset.
3. Select a scenario and press **Load / Reset**. This places the robot and
   scenario-owned props and configures the model with motion disabled.
4. Inspect the scene and prompt. Changes to the draft require another reset.
5. Press **Start & Record**. The manager requires a fresh prediction before
   enabling motion. **Finish & save run** ends it manually; arrival, timeout,
   Abort or an error also ends recording.
6. Open **Review & score**, enter your judgments and optional 0–100 score,
   and save the review. Export CSV from Recent results for analysis.

**Collisions are recorded and do not stop a scenario. Stopping is not scored.**
Arrival means the robot centre enters the evaluation region, with optional
heading tolerance and no speed/dwell requirement. Automatic measurements and
human reviews are stored separately.

For driving without recording, use **Enable motion (no recording)** in the
scenario panel or **Enable motion** on the model dashboard. Reset before the
next recorded trial. Camera changes and checkpoint switches require another
Load / Reset before recording.

The default filter shows A01–A20. Uncheck it to see the older 62 scenarios.
The study cases cover clear/obstructed approaches, mirrored corners, the
staggered-pallet maze, instructed bypasses, a gap, wall following, colour/object
selection and routes via landmarks. These are warehouse adaptations rather
than a reproduction of the paper's benchmark.

Use [the per-scenario condition tables](ablation-study-tables.md) or
[the CSV template](ablation-results-template.csv) for manual scoring. Each of
20 scenarios has 24 table conditions: Original/CAST × rear/front at 90°/120° ×
language/pose/language+pose. Pose-only does not receive route instructions, so
its instruction-adherence score is unscored. Keep the controller fixed when
comparing checkpoints, cameras or modalities.

### Edit scenarios

Edit `scenarios/warehouse.yaml` or use the existing scenario editor. Each entry
specifies ID/name, mode, prompt, starting robot pose, pose/image goal,
evaluation region/timeout, speed limits, props and optional route gates.
Positions are world metres; yaw fields are degrees. Supported prop kinds are
barrel, crate, pallet, cone, box, couch and USD. Procedural props need no asset
download. The manager only replaces props under `/World/Scenario`.

Route gates are passive scoring regions: they do not switch goals or repair
paths. The whole prompt remains unchanged during execution. The supplied
library requires `Simple_Warehouse/full_warehouse.usd` and kinematic motion.
Use a new study name after changing scenario geometry or prompts.

### Recorded files

Each trial is saved under `runtime/output/scenarios/<run-id>/`:

- `scenario.yaml`: complete scenario, model/step, camera and controller settings.
- `trajectory.jsonl`: timestamped poses, predictions and contact observations.
- `summary.json`: arrival, duration, distance, contact episodes and route gates.
- `start.jpg`, `end.jpg`, and an optional goal image.
- `review.json`: saved human judgments and notes.

Continuous per-trial video recording is not implemented. The review map shows
scenario props, so use camera images and the native viewer for warehouse shelves.
Runtime outputs and caches are excluded from Git.

## Model dashboard and checkpoints

On **http://127.0.0.1:8080/**, choose a modality, type an instruction and press
**Apply modality / instruction**. Language-only excludes pose/image goal
conditioning even if a goal remains stored or drawn. Use Capture image/pose or
relative forward/left/yaw fields for the corresponding goal-conditioned modes.
Changing prompts/goals/settings disables motion and invalidates in-flight results.

**Pause inference** disables motion and freezes the prediction; **Step once**
requests one new prediction while keeping motion off. The camera remains live.
Text is passed through unchanged. Wording and punctuation can affect model
outputs; the supplied language presets use fragments without a final period.

| Checkpoint | Startup `OMNIVLA_MODEL` | `OMNIVLA_RESUME_STEP` |
| --- | --- | ---: |
| Original | `omnivla-original` | 120000 |
| CAST finetuned | `omnivla-finetuned-cast` | 210000 |

The scenario panel can switch between downloaded checkpoints in the existing
model process. It unloads the previous predictor first; wait for loading and
reset before starting a trial. The displayed loaded path/step is authoritative.
This choice lasts until the model process restarts. Startup defaults can be
changed with the environment variables above; see the root guide.

## Camera and environment

The camera panel changes horizontal FOV and its offset relative to the URDF
mount while stopped. Rear/front presets keep height fixed; 90°/120° change FOV.
Save custom presets in the browser. Actual numerical settings are saved with
each trial. Reset after changing the camera.

Startup variables are `ISAAC_CAMERA_HFOV_DEG` (default 90) and
`ISAAC_CAMERA_OFFSET` (default `-0.8106,0,0.4875` m). The latter moves the camera
from its URDF mount to base-frame position `[-0.45,0.0175,0.95]` m.
The source does not automatically hide visible robot geometry.

`ISAAC_ENVIRONMENT` defaults to `Simple_Warehouse/full_warehouse.usd` under
NVIDIA's `Isaac/Environments/` asset root. `ISAAC_FILL_LIGHT` defaults to 500.
`ISAAC_SPAWN=auto` chooses the map's startup pose; set `x,y,yaw_deg` to override.
Managed scenario resets apply their own starting poses. Other environments
are usable for manual driving, but not with this warehouse scenario library.

## Trajectory evaluation mode

`ISAAC_DRIVE_MODE=kinematic` is the default. The bridge sends the eight predicted
local poses, their observation origin and the scenario's speed limits.
`OMNIVLA_KINEMATIC_CONTROLLER=waypoint` selects the fifth point by default
(`OMNIVLA_WAYPOINT_INDEX=4`, zero based).

In `husky_controller.py`, the selected metric point becomes `v=x/DT` and
`w=atan(y/x)/DT`, with `DT=1/3 s`. Initial forward/yaw clipping and the scenario
limits are followed by acceleration ramps. Translating points use their position
bearing; the heading vector is normally ignored. Isaac receives direct body
velocities, bypassing wheel forces while keeping obstacle contacts. The command
is held until replaced; this mode does not track the entire displayed curve or
continuously correct toward a fixed world waypoint.

The bridge requests inference at `OMNIVLA_INFERENCE_RATE=1` Hz of wall time.
Actual frequency also depends on model latency and camera availability.
To change frequency or selected point, stop motion and recreate only the bridge:

```bash
OMNIVLA_INFERENCE_RATE=2 OMNIVLA_WAYPOINT_INDEX=4   docker compose -f isaac_sim/compose.yaml --profile omnivla up -d --no-deps bridge
```

`OMNIVLA_KINEMATIC_CONTROLLER=trajectory` instead interpolates all predicted
positions/headings in world coordinates. This is ideal planar pose playback,
including possible lateral/backward motion, rather than differential-drive
tracking. Contacts can still block it. No planner, obstacle avoidance or
footprint-based path correction is added by either option.

`ISAAC_DRIVE_MODE=physics` selects wheel-force control instead. The wheel radius,
track, limits and acceleration policy come from `assets/control.yaml`.
Torque, drive damping and tyre friction are explicit simulation assumptions,
not a calibrated real-Husky model. Keep the same drive-mode setting for the
simulator, model and bridge. Disable motion before recreating them together.

## Source and transport

| File | Responsibility |
| --- | --- |
| `compose.yaml`, `Dockerfile` | Images, user IDs, GPU/network access and persistent mounts |
| `live_husky.py` | Simulation, camera/odometry, body/wheel actuation and local HTTP API |
| `husky_controller.py` | Velocity conversion/limiting, watchdog and optional pose playback |
| `scenario_manager.py` | Scene resets, passive measurements and trial/review storage |
| `scenarios/warehouse.yaml` | Editable scenario definitions |
| `drive.html`, `web-viewer.Dockerfile` | Controls and NVIDIA's native viewer |
| `run_husky.py` | Optional URDF export and finite stationary camera capture |

Isaac and the bridge use ROS 2 Jazzy/FastDDS in domain 84, with host networking
and UDPv4 transport. The camera is RGB 640×480 at 5 Hz of simulation time;
odometry is simulation ground truth, not an EKF estimate. Topics are
`/a200_0000/sensors/camera_0/color/image`,
`/a200_0000/platform/odom/filtered`, and `/a200_0000/cmd_vel`.
The model RPC uses localhost 5555. Freshness checks, matching configuration/reset
IDs, and a 0.5 s controller heartbeat timeout gate actuation.
