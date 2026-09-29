"""
What every robot in this project has to be.

Seven methods and three honest flags:

    class MyArm(Robot):
        has_ik = False              # say what you can do; the cell adapts
        def connect(self): ...
        def disconnect(self): ...
        def tool_pose(self): ...
        def flange_pose(self): ...
        def joints(self): ...
        def move(self, pose=None, joints=None, kind="joint", vel=None, label=""): ...
        def stop(self): ...

The flags matter more than they look. An arm with `has_ik = False` still runs
the whole cycle -- the cell sends bare poses and loses only the early
"unreachable" refusal. That is what makes "works with any robot" honest
instead of "works with any robot that resembles an FR5".
"""

from __future__ import annotations

from core import CellError, Pose


class Robot:
    name = "robot"

    #: Can the controller turn a pose into joint angles for us? With IK the
    #: cell refuses an unreachable point BEFORE the arm starts moving.
    has_ik = False

    #: Can something interrupt a move already in progress? Without it, stop()
    #: means "stop after this move", and the cell says so rather than pretending.
    can_stop_mid_move = False

    #: MoveL as well as MoveJ. A sniffer inserted into a service port wants a
    #: straight line; an arm without one gets a joint move and a warning.
    can_move_linear = False

    #: Does this controller expose digital I/O we can use for the leak tester?
    has_io = False

    # -- lifecycle ---------------------------------------------------------
    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def ready(self) -> list[str]:
        """Reasons this arm will refuse to move, in plain sentences. Empty is
        good. Never raises -- an unreadable controller is itself an answer."""
        return []

    def prepare(self) -> None:
        """Put the controller into a state where it accepts motion: clear
        faults, automatic mode, servos on. Must be safe to call twice."""

    def release(self) -> None:
        """Undo prepare(): back to manual, servos as the operator expects."""

    def verify(self) -> list[str]:
        """Checks that need the arm connected but move nothing. Empty is good."""
        return []

    # -- reading -----------------------------------------------------------
    def tool_pose(self) -> Pose:
        """Where the TIP is, in the base frame."""
        raise NotImplementedError

    def flange_pose(self) -> Pose:
        """Where the FLANGE is, in the base frame. The hand-eye chain needs
        this one, not the tool pose -- the camera is bolted to the flange."""
        raise NotImplementedError

    def joints(self) -> tuple:
        raise NotImplementedError

    def joint_limits(self):
        """[(min, max)] per joint in degrees, or None if the controller will
        not say. Worth the round trip: it is the difference between 'error 154'
        and 'joint 3 would need -158 and it stops at -150'."""
        return None

    # -- kinematics --------------------------------------------------------
    def solve(self, pose: Pose, seed=None) -> tuple:
        """Pose -> joint angles. Only called when has_ik is True."""
        raise NotImplementedError

    # -- motion ------------------------------------------------------------
    def move(self, pose=None, joints=None, kind="joint", vel=None, acc=None,
             blend_mm=None, label="") -> None:
        raise NotImplementedError

    def stop(self) -> None:
        """Halt motion now. NOT an emergency stop -- the E-stop is the button."""
        raise NotImplementedError

    def go_home(self, vel=None) -> None: ...

    def set_speed(self, percent: float) -> None: ...

    # -- I/O, for the sniffer's trigger and ready lines --------------------
    def read_input(self, index: int) -> int:
        raise CellError(f"{self.name} has no digital inputs")

    def write_output(self, index: int, value) -> None:
        raise CellError(f"{self.name} has no digital outputs")

    # -- misc --------------------------------------------------------------
    def describe(self) -> str:
        can = [n for n, f in (("IK", self.has_ik),
                              ("stop mid-move", self.can_stop_mid_move),
                              ("linear moves", self.can_move_linear),
                              ("digital I/O", self.has_io)) if f]
        return f"{self.name} [{', '.join(can) if can else 'basic motion only'}]"

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.disconnect()
