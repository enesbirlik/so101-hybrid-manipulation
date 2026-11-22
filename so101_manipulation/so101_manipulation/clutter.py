"""Other cubes as obstacles for the jaws (no ROS).

A pick of K2 failed on 2026-11-22 with K4 standing 4.7 cm away (centre to centre): the jaws open
to a ~53 mm gap, so a jaw pad lands ~27-34 mm from the cube's centre, where the neighbour's face
was (47 - 15 = 32 mm). The planner now treats the other cubes it knows as obstacles:

- Every other cube is an upright box: its square footprint (edge, yaw) from the table up to its
  top (a stacked cube's top is n edges up).
- The hand's footprint at one arm pose and gripper angle: the mesh points of the jaws and the
  wrist camera (`CLUTTER_LINKS`, gripper_link = body + fixed jaw) that lie below the obstacle's
  top + margin, projected on the table, one convex hull per link (the two jaws separately, so the
  gap between them stays free). Only the part of the hand low enough to touch the box counts,
  so a jaw passing over a cube is fine.
- Clearance = the smallest 2-D distance between those hulls and the obstacle's square, over every
  pose of the move (hover, descent, grasp); negative = overlap.

The node picks, among the closing directions the planner found, one whose clearance is at least
`clutter_margin_mm`; if none, a smaller opening (down to cube + 2 x `clutter_min_side_mm`); else it
refuses and names the neighbour. Place checks the jaws at the hold angle and at the release
opening, and the placed cube itself.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .kinematics import ArmModel, link_vertices

CLUTTER_LINKS = ("gripper_link", "moving_jaw_so101_v1_link", "wrist_camera_mount_link", "wrist_camera_link")
REACH_M = 0.16      # obstacles farther than this from the target (centre to centre) cannot meet the hand


@dataclass
class Obstacle:
    name: str
    xy: np.ndarray            # centre, base_link [m]
    yaw: float                # rad
    top: float                # top above the table [m] (edge for a cube on the table, 2 edges for a 2-stack)
    edge: float = 0.03

    def square(self, grow: float = 0.0) -> np.ndarray:
        """Footprint corners (4, 2), counter-clockwise, each side moved out by `grow`."""
        h = self.edge / 2 + grow
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        R = np.array([[c, -s], [s, c]])
        return np.array([[-h, -h], [h, -h], [h, h], [-h, h]]) @ R.T + np.asarray(self.xy, float)[:2]

    def describe(self, from_xy=None) -> str:
        s = f"{self.name} at ({self.xy[0] * 100:.1f}, {self.xy[1] * 100:.1f}) cm"
        if from_xy is not None:
            s += f", {np.linalg.norm(np.asarray(self.xy[:2]) - np.asarray(from_xy[:2])) * 100:.1f} cm away"
        if self.top > self.edge * 1.5:
            s += f", {self.top * 100:.0f} cm tall"
        return s


def square(xy, yaw, edge) -> np.ndarray:
    return Obstacle("", np.asarray(xy, float), float(yaw), edge, edge).square()


def hull2d(P: np.ndarray) -> np.ndarray:
    """Convex hull (counter-clockwise) of 2-D points, monotone chain; works for 1-2 points too."""
    P = np.asarray(P, float)
    if len(P) > 8:
        try:
            from scipy.spatial import ConvexHull
            return P[ConvexHull(P).vertices]        # counter-clockwise for 2-D input
        except Exception:  # noqa: BLE001  (degenerate: all on a line)
            pass
    P = np.unique(np.round(P, 7), axis=0)
    if len(P) <= 2:
        return P
    P = P[np.lexsort((P[:, 1], P[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in P:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in P[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def _seg_dist(P, A, B):
    """Distances from points P (n, 2) to segments A[i] - B[i] (m, 2): (n, m)."""
    AB = B - A
    L2 = np.maximum(np.sum(AB ** 2, axis=1), 1e-18)
    t = np.clip(((P[:, None, :] - A[None]) * AB[None]).sum(-1) / L2[None], 0.0, 1.0)
    C = A[None] + t[..., None] * AB[None]
    return np.linalg.norm(P[:, None, :] - C, axis=-1)


def poly_distance(A: np.ndarray, B: np.ndarray) -> float:
    """Distance between two convex polygons (counter-clockwise vertices) [m]; <= 0 when they overlap
    (then minus the smallest separating-axis overlap: a lower bound of the penetration depth)."""
    A, B = np.atleast_2d(A), np.atleast_2d(B)
    sep = -math.inf
    for P, Q in ((A, B), (B, A)):
        if len(P) < 3:
            continue
        E = np.roll(P, -1, axis=0) - P
        N = np.c_[E[:, 1], -E[:, 0]]            # outward normals of a counter-clockwise polygon
        N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-18)
        s = np.min(((Q[None, :, :] - P[:, None, :]) * N[:, None, :]).sum(-1), axis=1)   # per edge
        sep = max(sep, float(np.max(s)))
    if len(A) >= 3 and len(B) >= 3 and sep <= 0:
        return sep
    d = math.inf
    for P, Q in ((A, B), (B, A)):
        Qa, Qb = (Q, np.roll(Q, -1, axis=0)) if len(Q) > 1 else (Q, Q)
        d = min(d, float(np.min(_seg_dist(P, Qa, Qb))))
    return d


class HandFootprint:
    """Mesh points of the hand (jaws and wrist camera) in their links, thinned to a `voxel` grid."""

    def __init__(self, model: ArmModel, links=CLUTTER_LINKS, voxel: float = 0.002):
        self.m = model
        self.parts = []
        for name, chain, hull in model.links:
            if name not in links:
                continue
            try:
                src = model.camera_xacro if name.startswith("wrist_camera") else model.urdf
                P = link_vertices(src, name, model.mesh_dir)
            except Exception:  # noqa: BLE001  (xacro links: the hull is all there is)
                P = hull
            _, idx = np.unique(np.floor(P / voxel), axis=0, return_index=True)
            self.parts.append((name, chain, np.vstack([P[idx], hull])))

    def world(self, q_arm, grip):
        """[(link, points (n, 3) in base_link)]."""
        q = self.m.qdict(q_arm, grip)
        out = []
        for name, chain, P in self.parts:
            T = chain.forward(q)
            out.append((name, P @ T[:3, :3].T + T[:3, 3]))
        return out

    def clearance(self, poses, grip, obstacles, margin_z: float = 0.005):
        """(clearance [m], obstacle, link, pose index) of the closest approach of the hand at gripper angle
        `grip` [rad], over the arm `poses`, to any of `obstacles` (upright boxes from the table to their
        top). Only hand points below an obstacle's top + margin_z count. (inf, None, None, -1) if nothing
        comes low enough."""
        best = (math.inf, None, None, -1)
        if not obstacles:
            return best
        sq = [(o, o.square(), self.m.table_z + o.top + margin_z) for o in obstacles]
        for i, q in enumerate(poses):
            for link, W in self.world(q, grip):
                for o, S, zcut in sq:
                    low = W[W[:, 2] < zcut]
                    if len(low) == 0:
                        continue
                    # quick reject: the whole link is far from the box
                    c = np.asarray(o.xy[:2], float)
                    if np.min(np.linalg.norm(low[:, :2] - c, axis=1)) - o.edge * 0.71 > best[0]:
                        continue
                    d = poly_distance(hull2d(low[:, :2]), S)
                    if d < best[0]:
                        best = (d, o, link, i)
        return best


def near(obstacles, xy, reach=REACH_M):
    """The obstacles whose centre is within `reach` of xy (the others cannot meet the hand)."""
    xy = np.asarray(xy, float)[:2]
    return [o for o in obstacles if np.linalg.norm(np.asarray(o.xy[:2]) - xy) <= reach]


def min_open_deg(edge_mm, side_mm, gap_zero_mm, gap_per_deg_mm):
    """Gripper angle of a jaw gap of edge + 2 x side (the smallest opening the clutter fallback uses)."""
    return (edge_mm + 2 * side_mm - gap_zero_mm) / gap_per_deg_mm


def make_accept(footprint: HandFootprint, obstacles, grips_deg, margin, edge=0.03, release_deg=None, cube_xy=None):
    """accept(plan) -> (ok, why) for GraspPlanner.plan_* (`accept=`): the hand (jaws and camera below each
    obstacle's top + margin) stays `margin` [m] clear of every obstacle over the whole move (plan.poses()),
    at every gripper angle in grips_deg. Place (`release_deg`: release openings to try, largest first):
    the placed cube (at `cube_xy`, default the plan's centre, turned like the closing direction) is clear
    too, and plan.release_deg = the largest release opening that clears. Sets plan.clutter_m /
    clutter_with; counts rejections in accept.stats ("rejected", "best" = (clearance, obstacle, part) of the
    least bad rejected plan)."""
    stats = {"rejected": 0, "best": None, "plans": []}

    def note(c, ob, plan=None, what="the hand"):
        stats["rejected"] += 1
        if stats["best"] is None or c > stats["best"][0]:
            stats["best"] = (c, ob, what)
        if plan is not None:
            stats["plans"].append(plan)

    def accept(plan):
        poses = plan.poses()
        worst = (math.inf, None, None)
        if release_deg is not None:
            xy = plan.centre_xy if cube_xy is None else cube_xy
            sq = square(xy, plan.close_yaw, edge)
            for o in obstacles:
                d = poly_distance(sq, o.square())
                if d < worst[0]:
                    worst = (d, o, "the placed cube")
            if worst[0] < margin:
                note(worst[0], worst[1], what="the placed cube")
                return False, f"the cube would land {worst[0] * 1000:+.0f} mm from {worst[1].name}"
        for g in grips_deg:
            c, o, link, _ = footprint.clearance(poses, math.radians(g), obstacles, margin)
            if c < worst[0]:
                worst = (c, o, link)
        if worst[0] < margin:
            note(worst[0], worst[1], plan)
            part = "camera" if str(worst[2]).startswith("wrist_camera") else "jaws"
            return False, f"{part} {worst[0] * 1000:+.0f} mm from {worst[1].name} (margin {margin * 1000:.0f})"
        if release_deg is not None:
            c, o = math.inf, None
            for r in release_deg:
                c, o, link, _ = footprint.clearance(poses, math.radians(r), obstacles, margin)
                if c >= margin:
                    plan.release_deg = r
                    if c < worst[0]:
                        worst = (c, o, link)
                    break
            else:
                note(c, o, what="the jaws opening to release")
                return False, f"the jaws cannot open to release: {c * 1000:+.0f} mm from {o.name}"
        plan.clutter_m = worst[0]
        plan.clutter_with = worst[1].name if worst[1] is not None else ""
        return True, ""
    accept.stats = stats
    return accept


def openings(open_deg, min_deg, obstacles):
    """Jaw openings to try [deg], largest first: open_deg; with obstacles near also the middle and
    min_deg (cube + 2 x clutter_min_side_mm)."""
    o = float(open_deg)
    if not obstacles or min_deg >= o - 0.5:
        return [o]
    return [o, round((o + min_deg) / 2, 1), round(min_deg, 1)]


def refusal(what, xy, accepts, margin, min_deg):
    """Message naming the neighbour that blocked the least bad option, or '' if none did."""
    best = None
    for a in accepts:
        b = a.stats["best"]
        if b is not None and (best is None or b[0] > best[0]):
            best = b
    if best is None:
        return ""
    c, o, part = best
    return (f"{what}: {o.describe(xy)} is in the way: {part} comes within {c * 1000:+.0f} mm of it "
            f"(margin {margin * 1000:.0f} mm) for every closing direction"
            + (f", also with the jaws opened only to {min_deg:.1f} deg" if len(accepts) > 1 else ""))


def could_clear(footprint, plans, obstacles, open_deg, from_deg, margin, slack=0.002):
    """Can a smaller opening `open_deg` clear where `plans` (rejected at `from_deg`) did not? Checks the
    same arm poses at the smaller angle: re-planned, the hand moves by at most half the change of the
    jaw gap (the gap midpoint stays on the cube), so a clearance below margin - that - slack is hopeless."""
    if not plans:
        return True
    shift = abs(from_deg - open_deg) * 1.35e-3 / 2 + slack
    for p in plans:
        c, *_ = footprint.clearance(p.poses(), math.radians(open_deg), obstacles, margin)
        if c >= margin - shift:
            return True
    return False


def plan_among(plan_fn, footprint, obstacles, open_deg, min_deg, margin, edge=0.03):
    """Grasp among other cubes: plan_fn(open_deg, accept) -> (plan, msg) is the planner at one jaw
    opening. Tries open_deg, then smaller openings while the neighbours are what rejects the plans.
    (plan with plan.open_deg, '', accepts) or (None, why, accepts)."""
    accepts, msg = [], ""
    for o in openings(open_deg, min_deg, obstacles):
        if accepts and not could_clear(footprint, accepts[-1].stats["plans"], obstacles, o, open_deg, margin):
            continue                # cheap check: the hand would still be far too close (saves a full plan)
        acc = make_accept(footprint, obstacles, [o], margin, edge) if obstacles else None
        plan, msg = plan_fn(o, acc)
        if acc is not None:
            accepts.append(acc)
        if plan is not None:
            plan.open_deg = o
            return plan, "", accepts
        if acc is None or acc.stats["rejected"] == 0:
            break                   # not the neighbours (out of reach etc.): a smaller opening will not help
    return None, msg, accepts
