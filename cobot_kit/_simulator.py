"""
An arm made of arithmetic.

Not a toy: it is the reason the vision half, the planner and main.py can be
written and tested before anyone books the cell. It obeys a reach limit,
refuses poses it cannot make, takes time proportional to distance, and can be
stopped mid-move -- so code that works here is code that has already met the
failure modes it will meet on the FR5.

It has no real kinematics. `solve()` returns a made-up-but-deterministic set of
joint angles, and `has_ik` is True only so the reachability check is exercised.
"""

from __future__ import annotations

import math
import threading
import time

from .datatypes import Pose, Refused, Unreachable

from ._robot_base import Robot


def _wrap(deg: float) -> float:
    return (float(deg) + 180.0) % 360.0 - 180.0


class Simulator(Robot):
    """
        Simulator()                        a generic 900 mm arm
        Simulator(speed_factor=0.0)        instant moves, for tests
    """

    name = "simulator"
    has_ik = True
    can_stop_mid_move = True
    can_move_linear = True
    has_io = True

    def __init__(self, reach_mm=900.0, start=(600.0, -300.0, 200.0, 180.0, 0.0, 175.0),
                 home=None, tip_length_mm=157.0, speed_factor=0.002, quiet=False,
                 ip=None, tool=None, user=None, speed=None, auto_mode=None):
        # ip / tool / user / speed / auto_mode are accepted and ignored on
        # purpose. They mean nothing to an arm made of arithmetic, but they let
        # you switch ROBOT["kind"] from "fairino" to "simulator" by editing ONE
        # word instead of deleting five lines and putting them back afterwards.
        # A key that is not a real robot setting at all still raises, so a typo
        # is still caught.
        self.ignored = {k: v for k, v in
                        (("ip", ip), ("tool", tool), ("user", user),
                         ("speed", speed), ("auto_mode", auto_mode))
                        if v is not None}
        self.reach_mm = float(reach_mm)
        self.pose = Pose.of(start)
        self.home = list(home) if home else [0.0, -20.0, -90.0, -70.0, 90.0, 0.0]
        self.tip_length_mm = float(tip_length_mm)
        self.speed_factor = float(speed_factor)
        self.quiet = quiet
        self._joints = tuple(self.home)
        self._stop = threading.Event()
        self.log: list[str] = []
        self.speed = 100.0
        self.io = {}

    # -- lifecycle ---------------------------------------------------------
    def connect(self) -> None:
        if not self.quiet:
            print(f"  simulator: {self.reach_mm:.0f} mm reach, "
                  f"starting at {self.pose}")
            if self.ignored:
                print(f"  simulator: ignoring "
                      f"{', '.join(sorted(self.ignored))} -- real-robot "
                      f"settings that mean nothing here")

    def disconnect(self) -> None: ...

    # -- reading -----------------------------------------------------------
    def tool_pose(self) -> Pose:
        return self.pose

    def flange_pose(self) -> Pose:
        """The flange is tip_length back along the tool's own z axis."""
        from .geometry import tool_axis
        axis = tool_axis(self.pose)
        return self.pose.at([c - a * self.tip_length_mm
                             for c, a in zip(self.pose.xyz, axis)])

    def joints(self) -> tuple:
        return self._joints

    def joint_limits(self):
        return [(-175.0, 175.0), (-265.0, 85.0), (-160.0, 160.0),
                (-265.0, 85.0), (-175.0, 175.0), (-175.0, 175.0)]

    # -- kinematics --------------------------------------------------------
    def solve(self, pose: Pose, seed=None) -> tuple:
        if pose.reach > self.reach_mm:
            raise Unreachable(f"{pose} is {pose.reach:.0f} mm from the base and "
                              f"this arm reaches {self.reach_mm:.0f} mm")
        # Deterministic nonsense: repeatable, in range, and NOT real kinematics.
        base = math.degrees(math.atan2(pose.y, pose.x))
        r = math.hypot(pose.x, pose.y)
        return (_wrap(base),
                _wrap(-r / 12.0),
                _wrap(pose.z / 10.0),
                _wrap(pose.rx / 2.0),
                _wrap(pose.ry + 90.0),
                _wrap(pose.rz))

    # -- motion ------------------------------------------------------------
    def move(self, pose=None, joints=None, kind="joint", vel=None, acc=None,
             blend_mm=None, label="") -> None:
        if pose is None and joints is None:
            raise Refused("a move needs a pose, joints, or both")
        self._stop.clear()

        if pose is not None:
            if pose.reach > self.reach_mm:
                raise Unreachable(f"{pose} is out of reach "
                                  f"({pose.reach:.0f} > {self.reach_mm:.0f} mm)")
            distance = self.pose.distance_to(pose)
        else:
            distance = 200.0

        seconds = distance * self.speed_factor * (100.0 / max(1.0, vel or 20.0))
        end = time.time() + seconds
        while time.time() < end:
            if self._stop.is_set():
                self.log.append(f"stopped during {label or kind}")
                raise Refused(f"'{label or kind}' was stopped mid-move")
            time.sleep(0.005)

        if pose is not None:
            self.pose = pose
            self._joints = self.solve(pose)
        elif joints is not None:
            self._joints = tuple(float(v) for v in joints)
        self.log.append(f"{kind} -> {label or 'point'} {pose if pose else joints}")

    def stop(self) -> None:
        self._stop.set()
        self.log.append("stop")
        if not self.quiet:
            print("\n  simulator: stop")

    def go_home(self, vel=None) -> None:
        self.move(joints=self.home, kind="joint", vel=vel or 10.0, label="home")

    def set_speed(self, percent: float) -> None:
        self.speed = float(percent)

    # -- I/O ---------------------------------------------------------------
    def read_input(self, index: int) -> int:
        return int(self.io.get(int(index), 0))

    def write_output(self, index: int, value) -> None:
        self.io[int(index)] = int(bool(value))
