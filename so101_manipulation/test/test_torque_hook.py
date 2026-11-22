"""Torque-limit hook: publish until the readback matches, abort otherwise (stub node, no ROS graph)."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

pytest.importorskip("std_msgs")
from so101_manipulation.hooks import GripperTorqueLimitHook  # noqa: E402


class Log:
    def error(self, *_):
        pass

    info = warn = error


class Pub:
    def __init__(self, hw, subs=1):
        self.hw, self.subs, self.sent = hw, subs, []

    def get_subscription_count(self):
        return self.subs

    def publish(self, msg):
        self.sent.append(msg.data[0])
        self.hw.on_publish(msg.data[0])


class HW:
    """Readback follows the command only from the `lose`-th publish on (the first ones get lost)."""
    def __init__(self, lose=2, works=True):
        self.limit, self.lose, self.works, self.n = 1000.0, lose, works, 0

    def on_publish(self, v):
        self.n += 1
        if self.works and self.n > self.lose:
            self.limit = v

    def gripper_torque_limit(self):
        return self.limit


class Node:
    def __init__(self, hw, subs=1):
        self.hw, self.subs = hw, subs

    def create_publisher(self, *_):
        self.pub = Pub(self.hw, self.subs)
        return self.pub

    def get_logger(self):
        return Log()


def make(hw, subs=1, required=True):
    n = Node(hw, subs)
    return GripperTorqueLimitHook(n, hw, 200.0, 1000.0, timeout_s=0.6, required=required), n


def test_repeats_until_the_readback_matches():
    hw = HW(lose=2)
    h, n = make(hw)
    ok, why = h.lower()
    assert ok and hw.limit == 200.0 and len(n.pub.sent) >= 3
    h.restore()
    assert hw.limit == 1000.0 and not h.lowered


def test_aborts_when_it_never_applies():
    hw = HW(works=False)
    h, _ = make(hw)
    ok, why = h.lower()
    assert not ok and "did not apply" in why
    assert not h.lowered


def test_missing_interface_required_vs_mock():
    hw = HW()
    h, _ = make(hw, subs=0)
    assert not h.lower()[0]                 # real arm, interface missing: refuse
    assert h.lower(is_mock=True)[0]         # mock hardware: skipped
    h2, _ = make(hw, subs=0, required=False)
    assert h2.lower()[0]


def test_restore_only_after_lowering():
    hw = HW()
    h, n = make(hw)
    h.restore()
    assert n.pub.sent == []
