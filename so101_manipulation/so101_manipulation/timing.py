"""Per-phase wall time of one pick or place (no ROS)."""

from __future__ import annotations

import time


class PhaseTimer:
    """timer.enter("search") ... timer.enter("approach") ... timer.stop(); a phase entered twice
    (a grasp retry) adds up. totals() keeps the order in which the phases were first entered."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.t0 = clock()
        self.cur, self.since = None, self.t0
        self.acc = {}
        self.end = None

    def enter(self, phase: str) -> str:
        now = self.clock()
        if self.cur is not None:
            self.acc[self.cur] = self.acc.get(self.cur, 0.0) + now - self.since
        self.acc.setdefault(phase, 0.0)
        self.cur, self.since = phase, now
        return phase

    def stop(self):
        if self.end is None:
            now = self.clock()
            if self.cur is not None:
                self.acc[self.cur] = self.acc.get(self.cur, 0.0) + now - self.since
            self.cur, self.end = None, now

    def totals(self) -> dict:
        """{phase: s, ..., "total": s}, rounded to 0.1 s."""
        out = dict(self.acc)
        if self.cur is not None:
            out[self.cur] = out.get(self.cur, 0.0) + self.clock() - self.since
        out["total"] = (self.end if self.end is not None else self.clock()) - self.t0
        return {k: round(v, 1) for k, v in out.items()}

    def summary(self) -> str:
        """'search 2.1, approach 4.0, ..., total 28.3' (seconds)."""
        return ", ".join(f"{k} {v:.1f}" for k, v in self.totals().items())
