# so101_bt

> **Planned: phase 6** (Behavior Tree) and **phase 7** (hybrid orchestration & benchmark). Not a ROS package yet.

Behavior Tree orchestration of the full task, and the place where the research question lives.

Planned contents:

- Task trees for pick & place: perceive → plan (MoveIt 2) → **handover decision** → precise phase (policy) → verify.
- Handover conditions as swappable BT condition nodes (distance-based, confidence-based, policy-signalled), so the trigger can be varied per experiment.
- Recovery and fallback branches (re-perceive, retreat, retry).
- Logging of handover time and position for the benchmark in [docs/research/experiment-plan.md](../docs/research/experiment-plan.md).
