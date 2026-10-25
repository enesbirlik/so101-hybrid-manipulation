# Experiment plan (draft)

> Status: **draft**. This document will be refined before phase 7 and frozen before any benchmark data is collected.

## Research question

When, and under which condition, should control be handed over from a classical planner to a learned policy, and how does that handover affect success rate and task time?

## Conditions compared

| Condition | Description |
|---|---|
| **C — Classical only** | Perception gives a target pose; MoveIt 2 plans and executes the entire task. |
| **L — Learned only** | The VLA policy controls the arm from start to finish. |
| **H — Hybrid** | MoveIt 2 brings the gripper to a pre-grasp region; the Behavior Tree hands over to the policy for the precise phase. |

## Handover triggers (independent variable for H)

1. **Distance-based:** hand over when the end effector is within *d* cm of the estimated target (sweep *d*).
2. **Confidence-based:** hand over when perception confidence or pose uncertainty crosses a threshold.
3. **Learned / policy-driven:** hand over when the policy itself signals readiness (to be defined).

## Hypotheses

- **H1:** The hybrid condition reaches a success rate at least as high as *L*, at a lower mean task time.
- **H2:** The hybrid condition beats *C* in success rate on tasks with pose uncertainty or tight tolerances.
- **H3:** There is an intermediate handover distance that maximises success rate. Handing over too late hurts precision; handing over too early hurts speed and reliability.

## Metrics

- **Success rate:** binary per trial, with task-specific success criteria fixed in advance.
- **Task time:** from start command to success or timeout.
- **Handover time and position:** when and where the handover happened.
- **Failure modes:** categorised (perception miss, planning failure, grasp slip, collision, timeout).

## Protocol (to be detailed)

- Fixed task set (to be defined in phase 5–6), e.g. pick a small object and place it in a tight fixture.
- N trials per condition and trigger setting, with randomised object poses inside a defined workspace.
- Same hardware, lighting, and calibration across conditions. Every trial is logged as a rosbag (not committed to the repo).
- Results are reported with confidence intervals; raw per-trial results are stored as CSV.
