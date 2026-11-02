# OmniVLA-Husky

OmniVLA navigation experiments with a Clearpath Husky A200 in Isaac Sim.
Includes the ROS 2 bridge, model dashboard, editable warehouse scenarios,
camera/checkpoint controls, and manual ablation tables.

## Repository contents

| Directory | Purpose |
| --- | --- |
| `isaac_sim/` | Isaac Sim containers, browser viewer, Husky controller, scenario selector and evaluation |
| `bridge/` | ROS 2 camera/odometry transport and model command gate |
| `omnivla/` | Model Dockerfile and OmniVLA source with the integration's inference helpers |

The required Husky URDF, controller configuration and four referenced meshes are
included under `isaac_sim/assets/`. NVIDIA warehouse assets are fetched by Isaac
on first use. Checkpoints, datasets, caches, recordings, credentials and local
environment files are excluded from Git. No Git LFS or recursive clone is needed.

The ROS bridge uses Clearpath's published Jazzy development image from GHCR.
It does not require a local `clearpath_docker/` checkout or Gazebo.

## Prerequisites

Use a Linux workstation with Docker Engine, Docker Compose v2, NVIDIA drivers
and the NVIDIA Container Toolkit configured for GPU containers. This integration
has been exercised on an RTX 5090; the model image uses PyTorch 2.7.1 with CUDA
12.8, and the simulator image is pinned to Isaac Sim 6.0.0. Other GPUs have not
been validated here. Model inference and rendering share GPU memory.

Internet access is needed for Docker images, Python/Node packages, model weights
and the initial NVIDIA scene download. Running the Isaac image accepts NVIDIA's
EULA through the existing `ACCEPT_EULA=Y` setting in Compose.

Run the commands below from the repository root. Set these in each new terminal
so generated files belong to your account; defaults remain UID/GID 1001:

```bash
export USER_UID="$(id -u)" USER_GID="$(id -g)"
```

## Build and download the checkpoints

Create the writable bind-mount directories before starting containers:

```bash
mkdir -p omnivla/models "$HOME/.cache/huggingface"
mkdir -p isaac_sim/runtime/{output,omnivla,cache,compute-cache,logs,config,data,pkg,kit-cache,kit-data,hub-cache}

docker compose -f omnivla/compose.yaml build omnivla
docker compose -f isaac_sim/compose.yaml --profile omnivla build live viewer
```

The model image copies the included `omnivla/src/` source. You do not need to
clone OmniVLA separately. Docker ignore files keep weights and runtime caches
out of the build contexts.

Download the original checkpoint into the directory mounted by Isaac's model
service. This uses the built image without requesting a GPU. The pinned revision
matches the original weights checked in this workspace:

```bash
docker run --rm --user "$USER_UID:$USER_GID" \
  -e HF_HOME=/tmp/huggingface \
  -v "$PWD/omnivla/models:/models" \
  --entrypoint python omnivla:rtx5090 -c '
from huggingface_hub import snapshot_download
snapshot_download(
    "NHirose/omnivla-original",
    revision="e36a84d4923c041149d441f93f3bdb7092bb5f07",
    local_dir="/models/omnivla-original",
    ignore_patterns=["lora_adapter/*", "dist_head*"],
)
'
```

Optional CAST checkpoint, using the same inference-file selection:

```bash
docker run --rm --user "$USER_UID:$USER_GID" \
  -e HF_HOME=/tmp/huggingface \
  -v "$PWD/omnivla/models:/models" \
  --entrypoint python omnivla:rtx5090 -c '
from huggingface_hub import snapshot_download
snapshot_download(
    "NHirose/omnivla-finetuned-cast",
    revision="7d3744a72cd89218be4d223783f0742819b6c6db",
    local_dir="/models/omnivla-finetuned-cast",
    ignore_patterns=["lora_adapter/*", "dist_head*"],
)
'
```

Allow roughly 15 GB per checkpoint. Checkpoint usage follows the publishers'
terms; weights are downloaded from their repositories and are not redistributed
here.

## Start Isaac Sim, OmniVLA and the bridge

```bash
docker compose -f isaac_sim/compose.yaml --profile omnivla up -d live viewer omnivla bridge
```

Compose waits for Isaac to become healthy before loading the model. The first
start can take several minutes for shaders and scene dependencies.

- Simulator, driving controls and scenario selector: **http://127.0.0.1:8211/**
- OmniVLA camera feed, prompts and predictions: **http://127.0.0.1:8080/**

Open these in a browser on the workstation, including a browser inside a remote
desktop. WebRTC also requires direct access to its media/signaling ports;
forwarding only the web page through SSH does not forward the video connection.
See [the streaming guide](isaac_sim/README.md#live-viewing-and-driving).

Original step 120000 is the startup default. To start with CAST instead:

```bash
OMNIVLA_MODEL=omnivla-finetuned-cast OMNIVLA_RESUME_STEP=210000 \
  docker compose -f isaac_sim/compose.yaml --profile omnivla up -d live viewer omnivla bridge
```

Both downloaded checkpoints can also be selected in the scenario interface.
Motion starts disabled. Select a scenario, modality and camera; press
**Load / Reset**, inspect the scene, then **Start & Record** when ready.
Collisions are recorded without ending a trial; stopping is not scored.

The default controller selects waypoint five and converts its position into
forward speed and yaw rate, using kinematic body control. It does not track all
eight waypoints. See [controller behavior](isaac_sim/README.md#trajectory-evaluation-mode)
when interpreting results. Trial artifacts go into ignored `isaac_sim/runtime/`.
The [ablation tables](isaac_sim/ablation-study-tables.md) and
[CSV template](isaac_sim/ablation-results-template.csv) are included for scoring.

Stop these services without removing containers or data:

```bash
docker compose -f isaac_sim/compose.yaml --profile omnivla stop bridge omnivla viewer live
```

## Upstream sources and local development

This repository includes the OmniVLA source snapshot rather than a submodule,
retaining the integration helpers. Its original upstream clone may remain nested
in an existing development workspace; its `.git` directory is excluded.

- [OmniVLA](https://github.com/NHirose/OmniVLA),
  base commit `5182600cb4a9ee07684e17cdd2a6cbafc56b8a68`;
  [MIT license](omnivla/src/LICENSE).
- Husky/RealSense asset sources, notices and the mesh material adjustment are
  recorded in [the asset notices](isaac_sim/assets/NOTICE.txt).

Keep host-specific environment values in ignored `.env` files and local Compose
overrides. Model weights and results stay on disk when ignored; Git does not
remove them. Detailed operating instructions are in the
[Isaac guide](isaac_sim/README.md).
