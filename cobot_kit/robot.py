"""
The cobot, behind start() and stop().

    from cobot_kit import Robot

    robot = Robot()                  # values from settings.py
    robot = Robot(kind="simulator")  # no hardware
    robot.start()                    # connect, clear faults, AUTOMATIC, servos on,
                                     # verify the pendant tool, speed, home
    robot.move_joints([...])
    robot.move_to([x, y, z, rx, ry, rz])
    robot.write_output(0, 1)         # a DO on the controller
    robot.wait_input(0, 1, timeout_s=30)   # wait for a DI -> True / False
    robot.read_input(0)              # a DI on the controller (your trigger?)
    robot.stop()                     # halt, home, MANUAL, disconnect -- never raises

Guarantees carried over from the real-arm work:

  * stop() never raises, is safe twice, and safe without start().
  * start() failing half way puts the arm back in MANUAL before raising.
  * halt() stops the CURRENT move from another thread and stays connected
    (a stop button, not a shutdown).
  * The Fairino fixes (MoveJ/MoveL argument order, IK vs pendant tool,
    second connection for StopMotion, robot_state_pkg in three shapes)
    live in _fairino.py untouched.
"""

from __future__ import annotations

import atexit
import threading

from . import settings
from ._robot_base import Robot as _Driver
from .datatypes import CellError, Pose, Unreachable


def _build_driver(kind: str, options: dict) -> _Driver:
    if kind == "fairino":
        from ._fairino import Fairino
        cls = Fairino
    elif kind == "simulator":
        from ._simulator import Simulator
        cls = Simulator
    else:
        raise CellError(f"unknown robot kind {kind!r} -- use 'fairino' or 'simulator'")
    try:
        return cls(**options)
    except TypeError as e:
        raise CellError(f"{kind} does not accept these settings: {e}") from e


class Robot:
    """
        Robot()                                   settings.ROBOT
        Robot(kind="simulator")                   fake arm, same API
        Robot(ip="192.168.58.2", tool=1)          override any single value
        Robot(home_on_start=False, verify=False)
    """

    def __init__(self, kind=None, home_on_start=None, home_on_stop=None,
                 verify=None, **overrides):
        config = dict(settings.ROBOT)
        config.update(overrides)
        config.pop("kind", None)
        self.kind = kind or settings.ROBOT.get("kind", "simulator")
        self.home_on_start = (settings.HOME_ON_START if home_on_start is None
                              else home_on_start)
        self.home_on_stop = (settings.HOME_ON_STOP if home_on_stop is None
                             else home_on_stop)
        self.verify = settings.VERIFY_ON_START if verify is None else verify
        self.arm = _build_driver(self.kind, config)
        self._connected = False
        self._lock = threading.RLock()
        self._atexit = False

    # ------------------------------------------------------------ lifecycle
    @property
    def is_connected(self) -> bool:
        return self._connected

    def start(self) -> "Robot":
        """Bring the arm up. Twice is a no-op. Raises CellError with a sentence."""
        with self._lock:
            if self._connected:
                return self
            arm = self.arm
            print(f"\n  starting {arm.describe()}")
            try:
                arm.connect()
                arm.prepare()
                if self.verify:
                    problems = arm.verify()
                    if problems:
                        raise CellError(
                            "the controller is not set up the way this cell "
                            "expects:\n    " + "\n    ".join(problems)
                            + "\n  Fix it on the pendant, or Robot(verify=False)"
                              " if you are sure.")
                speed = getattr(arm, "speed", None)
                if speed is not None:
                    arm.set_speed(float(speed))
                if self.home_on_start:
                    print("  homing ...")
                    arm.go_home(vel=settings.HOME_VELOCITY)
            except BaseException:
                for step in (arm.release, arm.disconnect):
                    try:
                        step()
                    except Exception:
                        pass
                raise
            self._connected = True
            if not self._atexit:
                atexit.register(self.stop)      # last line of defence
                self._atexit = True
            print(f"  {arm.name} ready\n")
            return self

    def stop(self, home: bool | None = None) -> None:
        """Halt, (home), MANUAL, disconnect. Every step independent. Never raises."""
        with self._lock:
            if not self._connected:
                return
            self._connected = False
            arm = self.arm
            print(f"\n  stopping {arm.name} ...")
            try:
                arm.stop()
            except Exception as e:
                print(f"  !! could not halt motion: {e}")
            if self.home_on_stop if home is None else home:
                try:
                    arm.go_home(vel=settings.HOME_VELOCITY)
                except Exception as e:
                    print(f"  !! could not go home: {e}")
            try:
                arm.release()
            except Exception as e:
                print(f"  !! could not return to manual: {e}")
            try:
                arm.disconnect()
            except Exception as e:
                print(f"  !! could not disconnect: {e}")
            print(f"  {arm.name} stopped\n")

    def set_manual(self) -> None:
        """
        Hand the arm to the operator (MANUAL) and STAY connected -- for a manual
        calibration. set_automatic() takes it back. Unlike stop(), no homing
        and no disconnect.
        """
        arm = self._need()
        print(f"  {arm.name}: -> MANUAL")
        arm.release()

    def set_automatic(self) -> None:
        """Back from set_manual(): clear faults, AUTOMATIC, servos on."""
        arm = self._need()
        print(f"  {arm.name}: -> AUTOMATIC")
        arm.prepare()

    def halt(self) -> None:
        """Stop the move in progress, stay connected. Safe from another thread."""
        if not self._connected:
            return
        try:
            self.arm.stop()
        except Exception as e:
            print(f"  !! halt failed: {e}")

    def _need(self) -> _Driver:
        if not self._connected:
            raise CellError("the robot is not started -- call robot.start()")
        return self.arm

    # -------------------------------------------------------------- reading
    def pose(self) -> Pose:
        """Where the TIP is, base frame."""
        return self._need().tool_pose()

    def flange(self) -> Pose:
        """Where the FLANGE is, base frame -- what the hand-eye maths needs."""
        return self._need().flange_pose()

    def joints(self) -> tuple:
        return self._need().joints()

    # --------------------------------------------------------------- motion
    def move_to(self, pose, vel=None, linear=False, label="") -> None:
        """
        Move the TIP to a base-frame pose [x, y, z, rx, ry, rz].
        IK is solved first (when the arm has it), so an unreachable target is
        refused BEFORE anything moves -- raises Unreachable.
        """
        arm = self._need()
        pose = pose if isinstance(pose, Pose) else Pose.of(pose)
        solved = arm.solve(pose) if arm.has_ik else None
        arm.move(pose=pose, joints=solved, kind="linear" if linear else "joint",
                 vel=settings.VELOCITY if vel is None else vel, label=label)

    def move_joints(self, joints, vel=None, label="") -> None:
        self._need().move(joints=list(joints), kind="joint",
                          vel=settings.VELOCITY if vel is None else vel,
                          label=label)

    def go_home(self, vel=None) -> None:
        self._need().go_home(vel=settings.HOME_VELOCITY if vel is None else vel)

    def can_reach(self, pose) -> bool:
        """True/False without moving. True when the arm has no IK (cannot tell)."""
        arm = self._need()
        if not arm.has_ik:
            return True
        try:
            arm.solve(pose if isinstance(pose, Pose) else Pose.of(pose))
            return True
        except Unreachable:
            return False

    # ------------------------------------------------------------------ I/O
    def read_input(self, index: int) -> int:
        return int(self._need().read_input(index) or 0)

    def write_output(self, index: int, value) -> None:
        self._need().write_output(index, value)

    def wait_input(self, index: int, value: int = 1, timeout_s=None,
                   poll_s: float = 0.02, cancel=None) -> bool:
        """
        Wait until DI[index] == value. True when it came, False on timeout
        (timeout_s=None waits forever) or when cancel() returns True --
        pass e.g. cancel=lambda: not app.robot_is_connected so a Stop
        button is not stuck behind a tester that never answers.
        """
        import time
        arm = self._need()
        deadline = None if timeout_s is None else time.time() + float(timeout_s)
        while int(arm.read_input(index) or 0) != int(value):
            if deadline is not None and time.time() > deadline:
                return False
            if cancel is not None and cancel():
                return False
            time.sleep(poll_s)
        return True

    def describe(self) -> str:
        return self.arm.describe() + ("" if self._connected else " (not connected)")

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()
