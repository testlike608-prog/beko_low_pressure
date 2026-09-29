"""
The robot side of the cell, behind two functions.

    import robot

    robot.start()      connect, clear faults, AUTOMATIC, servos on, verify
                       the frames, set the speed, go home
    ... move it ...
    robot.stop()       halt motion, (optionally) home, back to MANUAL,
                       disconnect -- and never raise while doing it

That is the whole contract. Everything else in this module is a convenience on
top of those two, and every one of them is safe to call in any order:

    robot.start()      twice in a row is a no-op the second time
    robot.stop()       with nothing started is a no-op
    robot.stop()       twice is a no-op the second time

That matters more than it sounds. `stop()` is what runs in a `finally:`, in an
exception handler, on Ctrl+C, and from a GUI button -- often more than one of
those at the same instant. A stop that can itself fail is a stop you cannot
trust, so this one swallows every error, prints what happened, and keeps going
to the next shutdown step.

    robot.halt()       stop the CURRENT move and stay connected
                       (the run's stop button -- not a shutdown)

ADDING A ROBOT (three steps, one new file):

    1. copy robot/simulator.py to robot/myarm.py, fill in the seven methods
       and set the capability flags honestly
    2. add one line to REGISTRY below
    3. set ROBOT = {"kind": "myarm", ...} in settings.py

Nothing else -- not the camera, not the vision model, not main.py -- changes.
"""

from __future__ import annotations

import atexit
import threading

from core import CellError, Pose

from .base import Robot

# ---------------------------------------------------------------- registry
#: kind -> (module, class). Imported lazily, so a PC without the fairino SDK
#: still runs the simulator and the whole test suite.
REGISTRY = {
    "fairino": ("robot.fairino", "Fairino"),
    "simulator": ("robot.simulator", "Simulator"),
}


def kinds() -> list[str]:
    return sorted(REGISTRY)


def build(config: dict | None = None) -> Robot:
    """A config dict -> a Robot object, without connecting to anything."""
    if config is None:
        import settings
        config = settings.ROBOT
    config = dict(config)
    kind = config.pop("kind", "simulator")
    if kind not in REGISTRY:
        raise CellError(f"unknown robot kind {kind!r}. "
                        f"Known kinds: {', '.join(kinds())}")
    module_name, class_name = REGISTRY[kind]
    import importlib
    try:
        module = importlib.import_module(module_name)
    except ImportError as e:
        raise CellError(f"the {kind} driver could not be imported: {e}") from e
    try:
        return getattr(module, class_name)(**config)
    except TypeError as e:
        raise CellError(f"{kind} does not accept these settings: {e}") from e


# ------------------------------------------------------------- the singleton
_robot: Robot | None = None
_lock = threading.RLock()


def start(config: dict | None = None, home: bool | None = None,
          verify: bool = True, speed: float | None = None) -> Robot:
    """
    Bring the arm up and leave it ready to take motion commands.

        robot.start()                       everything from settings.py
        robot.start(home=False)             come up without homing
        robot.start(verify=False)           skip the frame check (faster, riskier)
        robot.start({"kind": "simulator"})  ignore settings.py entirely

    The steps, in this order and for a reason:

        connect     talk to the controller at all
        prepare     clear faults, AUTOMATIC, enable servos  <- announces itself
        verify      does the controller solve in the frames we command in?
                    This catches the wrong pendant tool BEFORE anything moves,
                    which is the single most expensive mistake in this cell.
        speed       global override, so a commissioning run is slow by config
        home        only after everything above passed

    If any step fails, whatever was already brought up is torn down again
    before the error is raised -- a half-started robot left in AUTOMATIC with
    the servos on is exactly the state nobody wants to walk into.
    """
    global _robot
    with _lock:
        if _robot is not None:
            return _robot

        import settings
        arm = build(config)
        print(f"\n  starting {arm.describe()}")

        try:
            arm.connect()
            arm.prepare()

            if verify:
                problems = arm.verify()
                if problems:
                    raise CellError(
                        "the controller is not set up the way this project "
                        "expects:\n    " + "\n    ".join(problems)
                        + "\n  Fix it on the pendant, or pass verify=False if "
                          "you are sure.")

            if speed is None:
                speed = getattr(settings, "SPEED_PERCENT", None)
            if speed is not None:
                arm.set_speed(float(speed))

            should_home = (getattr(settings, "HOME_ON_START", True)
                           if home is None else home)
            if should_home:
                print("  homing ...")
                arm.go_home(vel=getattr(settings, "HOME_VELOCITY", 10.0))

        except BaseException:
            # Never leave the arm live because start() failed halfway.
            try:
                arm.release()
            except Exception:
                pass
            try:
                arm.disconnect()
            except Exception:
                pass
            raise

        _robot = arm
        print(f"  {arm.name} ready\n")
        return arm


def stop(home: bool | None = None) -> None:
    """
    Stop everything and let go of the controller. Never raises.

        robot.stop()              motion halted, MANUAL, disconnected
        robot.stop(home=True)     go home first, then all of the above

    Each step is tried independently: a controller that will not accept a home
    move must still be put back in MANUAL, and one that will not switch mode
    must still be disconnected. Anything that fails is printed as a sentence
    and the next step runs anyway.

    This is NOT an emergency stop. The E-stop is the red button, and nothing in
    software replaces it.
    """
    global _robot
    with _lock:
        arm, _robot = _robot, None
        if arm is None:
            return

        print(f"\n  stopping {arm.name} ...")

        # 1. halt whatever is moving, first and unconditionally
        try:
            arm.stop()
        except Exception as e:
            print(f"  !! could not halt motion: {e}")

        # 2. home, only if asked -- a home move after a fault usually fails,
        #    and failing here must not block steps 3 and 4
        import settings
        should_home = (getattr(settings, "HOME_ON_STOP", False)
                       if home is None else home)
        if should_home:
            try:
                arm.go_home(vel=getattr(settings, "HOME_VELOCITY", 10.0))
            except Exception as e:
                print(f"  !! could not go home: {e}")

        # 3. hand the machine back to the operator
        try:
            arm.release()
        except Exception as e:
            print(f"  !! could not return to manual: {e}")

        # 4. drop the connection
        try:
            arm.disconnect()
        except Exception as e:
            print(f"  !! could not disconnect: {e}")

        print(f"  {arm.name} stopped\n")


def halt() -> None:
    """
    Stop the move in progress and STAY connected.

    This is the run's stop button, not a shutdown: after halt() the arm is
    still up and `robot.go(...)` works again. Safe from another thread, which
    is the whole point -- the drivers that support it open their own second
    connection, because the first one is blocked inside the move being
    interrupted.
    """
    arm = _robot
    if arm is None:
        return
    if not arm.can_stop_mid_move:
        print(f"  !! {arm.name} cannot be interrupted mid-move -- it will stop "
              f"when the current move finishes")
    try:
        arm.stop()
    except Exception as e:
        print(f"  !! halt failed: {e}")


def current() -> Robot:
    if _robot is None:
        raise CellError("the robot has not been started -- call robot.start()")
    return _robot


def is_running() -> bool:
    return _robot is not None


# ------------------------------------------------------------- conveniences
def pose() -> Pose:
    """Where the TIP is, base frame."""
    return current().tool_pose()


def flange() -> Pose:
    """Where the FLANGE is, base frame. This is what the hand-eye chain needs."""
    return current().flange_pose()


def joints() -> tuple:
    return current().joints()


def go(target, vel=None, kind="joint", label="") -> None:
    """
    Move to a pose. With IK available the joints are solved first and sent
    alongside, which is what the Fairino controller wants and what turns an
    unreachable target into a refusal BEFORE the arm starts moving.
    """
    arm = current()
    target = target if isinstance(target, Pose) else Pose.of(target)
    solved = None
    if arm.has_ik:
        solved = arm.solve(target)          # raises Unreachable, by design
    arm.move(pose=target, joints=solved, kind=kind, vel=vel, label=label)


def go_joints(values, vel=None, label="") -> None:
    current().move(joints=list(values), kind="joint", vel=vel, label=label)


def go_home(vel=None) -> None:
    current().go_home(vel=vel)


def can_reach(target) -> bool:
    """True / False without moving. Returns True when the arm has no IK -- it
    cannot tell, and pretending otherwise would drop good points."""
    arm = current()
    if not arm.has_ik:
        return True
    from core import Unreachable
    try:
        arm.solve(target if isinstance(target, Pose) else Pose.of(target))
        return True
    except Unreachable:
        return False


def describe(config: dict | None = None) -> str:
    if _robot is not None:
        return _robot.describe()
    try:
        return build(config).describe() + " (not connected)"
    except CellError as e:
        return f"robot misconfigured: {e}"


# A last line of defence: if the program exits in any way that skipped the
# finally block -- an unhandled exception, sys.exit, Ctrl+C at the wrong
# moment -- the arm is still put back in MANUAL and released.
atexit.register(stop)

__all__ = ["Robot", "REGISTRY", "kinds", "build",
           "start", "stop", "halt", "current", "is_running",
           "pose", "flange", "joints", "go", "go_joints", "go_home",
           "can_reach", "describe"]
