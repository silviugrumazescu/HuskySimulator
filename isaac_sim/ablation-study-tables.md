# OmniVLA warehouse ablation tables

Study name: __________  |  Evaluator: __________  |  Date: __________

Each scenario has its own table with all **24 conditions**: 2 checkpoints × 4 camera configurations × 3 modalities. Across 20 scenarios this is **480 runs per repeat**. CAST pose-only is included to keep the matrix complete.

Fill the result cells after each run. Repeat defaults to 1; duplicate rows for additional repeats. Record the run ID in Notes so the judgment can be traced to its saved recording.

## Camera configurations

Offsets are in metres relative to the configured URDF camera mount. The front/rear comparison uses the same height.

| Camera preset | Horizontal FOV | Offset X (forward) | Offset Y (left) | Offset Z (up) |
| --- | --- | --- | --- | --- |
| rear-90 | 90° | -0.8106 | 0 | 0.4875 |
| rear-120 | 120° | -0.8106 | 0 | 0.4875 |
| front-90 | 90° | 0 | 0 | 0.4875 |
| front-120 | 120° | 0 | 0 | 0.4875 |

## Recording results

- **Arrival:** pass / fail / unsure.
- **Contacts:** number of contact episodes from the recorder; use 0 for a collision-free run.
- **Route adherence:** pass / fail / unsure / N/A. Pose-only receives no route instruction, so its route cells are already N/A. Use N/A for other conditions without a route requirement.
- **Score:** your optional 0–100 rating; leave blank until evaluated. This is not assigned automatically.
- **Notes / run ID:** record the episode ID and observations such as wrong landmark, skipped detour, corner cutting or oscillation.

Collisions continue the trial. Stopping is not evaluated. Keep the controller, inference frequency, lighting, camera orientation and resolution fixed while comparing these factors. Use a new study name if scenario geometry or prompts change.

## Scenario index

| Scenario | Task |
| --- | --- |
| [A01](#a01) | Clear approach |
| [A02](#a02) | Central obstacle, either side |
| [A03](#a03) | Open left corner |
| [A04](#a04) | Open right corner |
| [A05](#a05) | Moderate left corner |
| [A06](#a06) | Moderate right corner |
| [A07](#a07) | Staggered pallet maze — right first |
| [A08](#a08) | Staggered pallet maze — left first |
| [A09](#a09) | Instructed left bypass |
| [A10](#a10) | Instructed right bypass |
| [A11](#a11) | Between boxes despite outside shortcut |
| [A12](#a12) | Follow wall, avoid open-floor shortcut |
| [A13](#a13) | Choose blue |
| [A14](#a14) | Choose red |
| [A15](#a15) | Blue barrel beside couch |
| [A16](#a16) | Blue barrel beside crate |
| [A17](#a17) | Direct control |
| [A18](#a18) | Via couch |
| [A19](#a19) | Via red barrel |
| [A20](#a20) | Via green barrel |

## A01

**Clear approach** (`study-01`)

Prompt: `move toward the blue barrel`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A01-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A01-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A01-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A01-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A01-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A01-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A01-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A01-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A01-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A01-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A01-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A01-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A01-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A01-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A01-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A01-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A01-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A01-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A01-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A01-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A01-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A01-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A01-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A01-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A02

**Central obstacle, either side** (`study-02`)

Prompt: `move toward the blue barrel`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A02-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A02-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A02-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A02-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A02-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A02-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A02-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A02-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A02-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A02-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A02-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A02-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A02-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A02-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A02-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A02-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A02-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A02-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A02-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A02-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A02-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A02-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A02-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A02-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A03

**Open left corner** (`study-03`)

Prompt: `follow the corridor toward the blue barrel`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A03-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A03-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A03-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A03-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A03-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A03-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A03-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A03-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A03-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A03-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A03-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A03-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A03-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A03-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A03-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A03-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A03-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A03-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A03-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A03-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A03-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A03-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A03-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A03-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A04

**Open right corner** (`study-04`)

Prompt: `follow the corridor toward the blue barrel`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A04-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A04-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A04-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A04-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A04-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A04-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A04-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A04-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A04-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A04-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A04-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A04-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A04-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A04-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A04-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A04-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A04-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A04-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A04-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A04-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A04-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A04-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A04-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A04-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A05

**Moderate left corner** (`study-05`)

Prompt: `follow the corridor toward the blue barrel`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A05-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A05-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A05-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A05-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A05-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A05-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A05-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A05-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A05-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A05-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A05-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A05-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A05-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A05-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A05-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A05-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A05-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A05-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A05-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A05-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A05-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A05-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A05-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A05-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A06

**Moderate right corner** (`study-06`)

Prompt: `follow the corridor toward the blue barrel`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A06-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A06-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A06-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A06-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A06-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A06-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A06-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A06-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A06-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A06-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A06-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A06-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A06-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A06-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A06-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A06-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A06-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A06-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A06-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A06-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A06-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A06-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A06-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A06-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A07

**Staggered pallet maze — right first** (`study-07`)

Prompt: `move toward the blue barrel`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A07-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A07-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A07-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A07-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A07-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A07-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A07-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A07-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A07-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A07-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A07-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A07-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A07-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A07-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A07-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A07-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A07-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A07-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A07-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A07-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A07-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A07-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A07-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A07-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A08

**Staggered pallet maze — left first** (`study-08`)

Prompt: `move toward the blue barrel`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A08-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A08-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A08-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A08-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A08-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A08-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A08-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A08-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A08-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A08-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A08-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A08-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A08-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A08-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A08-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A08-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A08-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A08-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A08-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A08-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A08-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A08-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A08-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A08-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A09

**Instructed left bypass** (`study-09`)

Prompt: `pass left of the crate toward the blue barrel`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A09-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A09-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A09-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A09-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A09-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A09-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A09-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A09-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A09-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A09-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A09-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A09-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A09-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A09-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A09-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A09-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A09-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A09-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A09-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A09-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A09-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A09-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A09-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A09-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A10

**Instructed right bypass** (`study-10`)

Prompt: `pass right of the crate toward the blue barrel`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A10-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A10-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A10-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A10-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A10-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A10-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A10-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A10-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A10-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A10-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A10-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A10-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A10-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A10-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A10-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A10-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A10-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A10-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A10-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A10-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A10-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A10-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A10-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A10-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A11

**Between boxes despite outside shortcut** (`study-11`)

Prompt: `pass between the two wooden boxes toward the blue barrel`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A11-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A11-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A11-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A11-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A11-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A11-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A11-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A11-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A11-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A11-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A11-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A11-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A11-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A11-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A11-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A11-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A11-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A11-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A11-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A11-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A11-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A11-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A11-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A11-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A12

**Follow wall, avoid open-floor shortcut** (`study-12`)

Prompt: `follow the wall toward the blue barrel`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A12-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A12-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A12-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A12-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A12-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A12-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A12-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A12-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A12-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A12-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A12-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A12-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A12-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A12-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A12-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A12-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A12-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A12-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A12-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A12-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A12-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A12-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A12-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A12-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A13

**Choose blue** (`study-13`)

Prompt: `move toward the blue barrel`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A13-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A13-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A13-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A13-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A13-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A13-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A13-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A13-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A13-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A13-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A13-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A13-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A13-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A13-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A13-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A13-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A13-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A13-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A13-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A13-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A13-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A13-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A13-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A13-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A14

**Choose red** (`study-14`)

Prompt: `move toward the red barrel`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A14-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A14-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A14-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A14-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A14-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A14-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A14-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A14-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A14-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A14-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A14-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A14-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A14-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A14-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A14-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A14-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A14-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A14-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A14-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A14-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A14-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A14-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A14-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A14-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A15

**Blue barrel beside couch** (`study-15`)

Prompt: `move toward the blue barrel beside the couch`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A15-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A15-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A15-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A15-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A15-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A15-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A15-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A15-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A15-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A15-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A15-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A15-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A15-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A15-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A15-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A15-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A15-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A15-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A15-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A15-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A15-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A15-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A15-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A15-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A16

**Blue barrel beside crate** (`study-16`)

Prompt: `move toward the blue barrel beside the crate`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A16-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A16-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A16-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A16-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A16-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A16-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A16-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A16-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A16-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A16-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A16-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A16-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A16-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A16-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A16-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A16-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A16-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A16-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A16-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A16-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A16-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A16-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A16-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A16-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A17

**Direct control** (`study-17`)

Prompt: `move toward the blue barrel`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A17-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A17-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A17-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A17-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A17-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A17-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A17-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A17-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A17-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A17-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A17-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A17-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A17-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A17-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A17-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A17-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A17-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A17-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A17-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A17-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A17-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A17-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A17-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A17-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A18

**Via couch** (`study-18`)

Prompt: `pass by the couch and then move toward the blue barrel`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A18-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A18-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A18-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A18-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A18-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A18-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A18-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A18-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A18-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A18-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A18-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A18-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A18-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A18-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A18-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A18-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A18-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A18-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A18-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A18-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A18-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A18-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A18-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A18-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A19

**Via red barrel** (`study-19`)

Prompt: `pass by the red barrel and then move toward the blue barrel`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A19-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A19-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A19-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A19-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A19-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A19-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A19-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A19-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A19-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A19-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A19-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A19-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A19-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A19-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A19-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A19-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A19-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A19-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A19-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A19-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A19-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A19-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A19-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A19-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________


## A20

**Via green barrel** (`study-20`)

Prompt: `pass by the green barrel and then move toward the blue barrel`

| Condition | Checkpoint | Camera preset | Modality | Repeat | Arrival | Contacts | Route adherence | Score (0–100) | Notes / run ID |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A20-01 | Original (120000) | rear-90 | Language | 1 |  |  |  |  |  |
| A20-02 | Original (120000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A20-03 | Original (120000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A20-04 | Original (120000) | rear-120 | Language | 1 |  |  |  |  |  |
| A20-05 | Original (120000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A20-06 | Original (120000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A20-07 | Original (120000) | front-90 | Language | 1 |  |  |  |  |  |
| A20-08 | Original (120000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A20-09 | Original (120000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A20-10 | Original (120000) | front-120 | Language | 1 |  |  |  |  |  |
| A20-11 | Original (120000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A20-12 | Original (120000) | front-120 | Language + pose | 1 |  |  |  |  |  |
| A20-13 | CAST (210000) | rear-90 | Language | 1 |  |  |  |  |  |
| A20-14 | CAST (210000) | rear-90 | Pose | 1 |  |  | N/A |  |  |
| A20-15 | CAST (210000) | rear-90 | Language + pose | 1 |  |  |  |  |  |
| A20-16 | CAST (210000) | rear-120 | Language | 1 |  |  |  |  |  |
| A20-17 | CAST (210000) | rear-120 | Pose | 1 |  |  | N/A |  |  |
| A20-18 | CAST (210000) | rear-120 | Language + pose | 1 |  |  |  |  |  |
| A20-19 | CAST (210000) | front-90 | Language | 1 |  |  |  |  |  |
| A20-20 | CAST (210000) | front-90 | Pose | 1 |  |  | N/A |  |  |
| A20-21 | CAST (210000) | front-90 | Language + pose | 1 |  |  |  |  |  |
| A20-22 | CAST (210000) | front-120 | Language | 1 |  |  |  |  |  |
| A20-23 | CAST (210000) | front-120 | Pose | 1 |  |  | N/A |  |  |
| A20-24 | CAST (210000) | front-120 | Language + pose | 1 |  |  |  |  |  |

Scenario observations: ____________________________________________________________

