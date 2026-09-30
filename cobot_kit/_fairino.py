"""
Fairino FR3 / FR5 / FR10 over the controller's own SDK.

Five things in here cost a day each to learn on the real arm, and none of them
are in the manual:

TWO LISTS, OPPOSITE ORDERS. MoveJ(joint_pos, tool, user, desc_pos=...) and
MoveL(desc_pos, tool, user, joint_pos=...) take the same two lists the other
way round. Copying one call to write the other puts the pose where the joints
belong, and python reports it as "got multiple values for argument
'joint_pos'" -- a message about keywords, for a swapped pair. So every
argument here is passed BY NAME and the ordering stops mattering.

THE PAIR MUST AGREE. Both calls hand the controller a joint target AND a pose,
and when the two describe different points it rejects the pair as a bad point
rather than as a mismatch.

GetInverseKin TAKES NEITHER A TOOL NOR A WORKPIECE. It solves in whatever the
pendant has active, while MoveJ/MoveL take both explicitly. When they differ
you get error 74 or 154 -- codes about joints and lines that say nothing about
frames. So `tool` and `user` here MUST match the pendant, and verify() checks
it before anything moves. It is the most valuable check in the driver.

A STOP NEEDS ITS OWN CONNECTION. Every motion call blocks, and one xmlrpc
proxy is not safe from two threads, so stop() opens a second one.

robot_state_pkg COMES BACK IN THREE SHAPES. Depending on the SDK build it is a
ctypes Structure instance, a POINTER to one, or the Structure CLASS itself. On
the last, every attribute is a field DESCRIPTOR, and int() on it raises
"int() argument must be ... not '_ctypes.CField'" -- which says nothing about a
robot and kills the run before the first move. _field() handles all three.
"""

from __future__ import annotations

import inspect
import time
import xmlrpc.client

from .datatypes import CellError, Pose, Refused, Unreachable

from ._robot_base import Robot

#: The controller's own numbers, in words. Anything not here is printed bare.
FAIRINO_ERRORS = {
    -7: "the command is not supported by this controller version",
    -4: "the controller is in an error state -- clear it on the pendant",
    -2: "no answer from the controller (link down)",
    -1: "the SDK rejected the parameters",
    8: "the arm is already moving",
    14: "a motion parameter is out of range",
    74: "inverse kinematics failed -- unreachable, or the wrong tool/workpiece",
    112: "the target is outside the joint soft limits",
    154: "the planned path is not executable -- out of reach or a singularity",
}


class Fairino(Robot):
    """
        Fairino(ip="192.168.57.2", tool=6, user=0, tip_length_mm=157)
    """

    name = "fairino"
    has_ik = True
    can_stop_mid_move = True
    can_move_linear = True
    has_io = True

    def __init__(self, ip="192.168.58.2", tool=0, user=0, home=None,
                 tip_length_mm=0.0, auto_mode=True, speed=30.0,
                 stop_port=20003, quiet=False):
        self.ip, self.tool, self.user = ip, int(tool), int(user)
        self.home = list(home) if home else None
        self.tip_length_mm, self.auto_mode = float(tip_length_mm), auto_mode
        self.speed, self.stop_port, self.quiet = float(speed), stop_port, quiet
        self.rpc = None
        self._limits, self._asked = None, False
        self.state_readable = True         # until ready() finds otherwise
        self.name = f"fairino@{ip}"

    # -- SDK plumbing ------------------------------------------------------
    @staticmethod
    def _value(ret, default=None):
        """The SDK returns either a bare error code or (code, value). Both."""
        if isinstance(ret, int):
            return default
        try:
            err, val = ret
        except (TypeError, ValueError):
            return default
        return val if err == 0 else default

    @staticmethod
    def _call(fn, **kw):
        """
        Call with every argument NAMED, dropping any this SDK build lacks.

        The dropping matters as much as the naming: `blendR` exists on some
        builds and not others, and a driver that hard-codes it works on one
        factory PC and raises TypeError on the next.
        """
        try:
            params = set(inspect.signature(fn).parameters)
            kw = {k: v for k, v in kw.items() if k in params}
        except (TypeError, ValueError):
            pass
        return fn(**kw)

    def _rpc(self):
        if self.rpc is None:
            raise CellError("the Fairino driver is not connected -- "
                            "call robot.start()")
        return self.rpc

    # -- lifecycle ---------------------------------------------------------
    def connect(self) -> None:
        if self.rpc is not None:
            return
        try:
            from fairino import Robot as FR
        except ImportError as e:
            raise CellError(
                "the fairino SDK is not importable. The project needs the "
                "'fairino' folder (with Robot.py) beside app.py -- run from "
                f"that folder, or check it was copied. ({e})"
            ) from e
        try:
            self.rpc = FR.RPC(self.ip)
        except Exception as e:
            raise CellError(
                f"no answer from the controller at {self.ip}. Check the cable, "
                f"the address, and that this PC is on the same subnet.") from e
        self.joint_limits()
        if not self.quiet:
            print(f"  fairino: connected to {self.ip}, tool {self.tool}, "
                  f"workpiece {self.user}")

    def disconnect(self) -> None:
        if self.rpc is not None:
            try:
                self.rpc.CloseRPC()
            except Exception:
                pass
            self.rpc = None

    # -- reading -----------------------------------------------------------
    def tool_pose(self) -> Pose:
        ret = self._rpc().GetActualTCPPose(0)
        if isinstance(ret, int):
            raise CellError(f"TCP read failed, SDK code {ret}")
        err, pose = ret
        if err != 0:
            raise CellError(f"TCP read error {err}")
        return Pose.of(pose)

    def flange_pose(self) -> Pose:
        ret = self._rpc().GetActualToolFlangePose(0)
        if isinstance(ret, int):
            raise CellError(f"flange read failed, SDK code {ret}")
        err, pose = ret
        if err != 0:
            raise CellError(f"flange read error {err}")
        return Pose.of(pose)

    def joints(self) -> tuple:
        vals = self._value(self._rpc().GetActualJointPosDegree(0))
        if vals is None:
            raise CellError("the controller would not report its joint angles")
        return tuple(float(v) for v in vals)

    def joint_limits(self):
        """Asked once and cached."""
        if self._asked:
            return self._limits
        self._asked = True
        try:
            flat = self._value(self._rpc().GetJointSoftLimitDeg(0))
            if flat and len(flat) >= 12:
                self._limits = [(float(flat[2 * i]), float(flat[2 * i + 1]))
                                for i in range(6)]
        except Exception:
            self._limits = None
        return self._limits

    # -- state -------------------------------------------------------------
    @staticmethod
    def _field(pkg, name, default=-1):
        """
        One field of the controller's status structure as an int, PLUS whether
        it was really read.

        The flag is the important half. An unreadable field comes back as the
        default AND as not-read, because "I cannot see the mode" and "the mode
        is fine" are different answers, and a driver that confuses them reports
        a ready robot that then refuses every motion.
        """
        if pkg is None:
            return default, False
        target = getattr(pkg, "contents", pkg)       # POINTER(...) -> the struct
        value = getattr(target, name, None)
        if value is None:
            return default, False
        try:
            return int(value), True
        except (TypeError, ValueError):              # a ctypes field descriptor
            return default, False

    def ready(self) -> list[str]:
        rpc = self._rpc()
        pkg = getattr(rpc, "robot_state_pkg", None)
        mode, mode_ok = self._field(pkg, "robot_mode")
        enabled, enabled_ok = self._field(pkg, "rbtEnableState")
        state, state_ok = self._field(pkg, "robot_state")
        collide, _ = self._field(pkg, "collisionState", 0)

        self.state_readable = any((mode_ok, enabled_ok, state_ok))

        estop = self._value(rpc.GetRobotEmergencyStopState())
        codes = self._value(rpc.GetRobotErrorCode(), [None, None])
        stops = self._value(rpc.GetSafetyStopState(), [None, None])

        problems = []
        if mode == 1:
            problems.append("the robot is in MANUAL mode -- SDK motion is only "
                            "accepted in AUTOMATIC")
        if enabled == 0:
            problems.append("the servos are not enabled")
        if state == 4:
            problems.append("the robot is in DRAG/TEACH mode -- it is held for "
                            "hand guiding and ignores motion commands")
        if collide:
            problems.append("a collision is flagged -- clear it before moving")
        if estop:
            problems.append("emergency stop is active")
        if stops and any(stops):
            problems.append(f"a safety stop is active {stops}")
        if codes and codes[0]:
            problems.append(f"the controller is holding fault {codes[0]}/{codes[1]}"
                            f" -- clear it on the pendant first")
        return problems

    def prepare(self) -> None:
        """
        Announced loudly on purpose. Automatic mode is a real change in how the
        machine behaves: the enabling switch stops gating movement and the arm
        moves whenever a command arrives. Anyone standing in the cell needs to
        learn that from the console, not from the arm.
        """
        rpc = self._rpc()
        problems = self.ready()
        if not problems and self.state_readable:
            rpc.SetSpeed(self.speed)
            return

        if problems:
            print("\n  the robot is not ready:")
            for p in problems:
                print(f"    !! {p}")
        else:
            # Nothing looks wrong -- but nothing can be SEEN either, and those
            # are not the same. Assuming automatic-and-enabled because the
            # status structure would not read is how a cell ends up refusing
            # every motion with a bare code instead of a sentence. So SET the
            # state rather than trust a silence. Every call below is idempotent
            # on a controller already in that state.
            print("\n  this SDK build does not report the controller's mode or "
                  "servo state as readable values, so the state cannot be "
                  "checked -- setting it instead of assuming it:")

        if not self.auto_mode:
            raise CellError("the controller will refuse every motion in this "
                            "state, and auto_mode is off in settings.py")

        print("\n  *** SWITCHING TO AUTOMATIC AND ENABLING THE SERVOS ***")
        print("  *** From here the arm moves on command, with no enabling "
              "switch. Nobody should be inside the cell. ***\n")
        for label, fn in (("ResetAllError", rpc.ResetAllError),
                          ("Mode(0) automatic", lambda: rpc.Mode(0)),
                          ("DragTeachSwitch(0)", lambda: rpc.DragTeachSwitch(0)),
                          ("RobotEnable(1)", lambda: rpc.RobotEnable(1)),
                          (f"SetSpeed({self.speed:.0f})",
                           lambda: rpc.SetSpeed(self.speed))):
            print(f"    {label:22s} -> {fn()}")
            time.sleep(0.4)
        time.sleep(1.0)

        problems = self.ready()
        if problems:
            raise CellError("still not ready:\n    " + "\n    ".join(problems))
        if self.state_readable:
            print("\n  robot ready")
        else:
            print("\n  automatic and enabled were commanded and accepted -- "
                  "this build cannot read the state back, so check the pendant "
                  "if a motion is refused")

    def release(self) -> None:
        if self.rpc is None:
            return
        try:
            self.rpc.StopMotion()
            time.sleep(0.2)
            self.rpc.Mode(1)
            if not self.quiet:
                print("  fairino: returned to MANUAL")
        except Exception as e:
            print(f"  !! could not return to manual: {e}")

    def verify(self) -> list[str]:
        """
        Check the frames the controller SOLVES in are the ones we command.
        See the module docstring -- this is the check that saves the most time.
        """
        problems = []
        tcp, flange = self.tool_pose(), self.flange_pose()
        gap = tcp.distance_to(flange)

        here = self.joints()
        solved = self._value(self._rpc().GetInverseKin(0, tcp.list(), -1))
        if solved is None:
            problems.append(
                "GetInverseKin will not solve the arm's CURRENT pose -- the "
                "pendant's active tool is probably not the one being commanded")
        else:
            drift = max(abs(a - b) for a, b in zip(solved, here))
            if drift > 1.0:
                problems.append(
                    f"the controller solves the arm's own pose to joints "
                    f"{drift:.1f} deg from where it is standing -- tool "
                    f"{self.tool} / workpiece {self.user} do not match the "
                    f"pendant's active ones")
        if self.tip_length_mm and abs(gap - self.tip_length_mm) > 5.0:
            problems.append(
                f"the tip is {gap:.1f} mm from the flange but settings.py says "
                f"{self.tip_length_mm:.1f} mm -- the pendant's tool is not the "
                f"one this cell was set up for")
        return problems

    # -- kinematics --------------------------------------------------------
    def solve(self, pose: Pose, seed=None) -> tuple:
        """
        Two solvers, because they fail differently. GetInverseKin(config=-1)
        picks a posture near the current one; when that comes back empty,
        GetInverseKinRef is asked the same question with a reference spelled
        out, which some firmware answers where the first refuses.
        """
        rpc = self._rpc()
        ret = rpc.GetInverseKin(0, pose.list(), -1)
        if isinstance(ret, int):
            raise CellError(f"GetInverseKin returned code {ret} (link down?)")
        err, sol = ret
        if err == 0 and sol is not None:
            return tuple(float(v) for v in sol)

        ref = list(seed) if seed else None
        if ref is None:
            try:
                ref = list(self.joints())
            except CellError:
                ref = None
        if ref is not None:
            ret = rpc.GetInverseKinRef(0, pose.list(), ref)
            if not isinstance(ret, int):
                err2, sol2 = ret
                if err2 == 0 and sol2 is not None:
                    return tuple(float(v) for v in sol2)

        raise Unreachable(f"no joint solution for {pose} (error {err}) -- out of "
                          f"reach, through the table, or a posture the joints "
                          f"cannot make")

    # -- motion ------------------------------------------------------------
    def move(self, pose=None, joints=None, kind="joint", vel=None, acc=None,
             blend_mm=None, label="") -> None:
        """
        Three attempts, each NARROWING what could be wrong rather than repeating
        the same call: the pair, then joints alone (which rules the pose out),
        then a straight line (which rules the joint-space path out). Whatever is
        left is the target itself.
        """
        rpc = self._rpc()
        if pose is None and joints is None:
            raise Refused("a move needs a pose, joints, or both")
        v = float(vel if vel is not None else 20.0)

        attempts = []
        if kind != "linear" and joints is not None:
            attempts.append(("MoveJ with the pose", dict(
                fn=rpc.MoveJ, joint_pos=list(joints), tool=self.tool,
                user=self.user, desc_pos=(pose.list() if pose else None),
                vel=v, acc=acc)))
            attempts.append(("MoveJ, joints only", dict(
                fn=rpc.MoveJ, joint_pos=list(joints), tool=self.tool,
                user=self.user, vel=v, acc=acc)))
        if pose is not None:
            attempts.append(("MoveL", dict(
                fn=rpc.MoveL, desc_pos=pose.list(), tool=self.tool,
                user=self.user,
                joint_pos=(list(joints) if joints is not None else None),
                vel=v, acc=acc,
                blendR=blend_mm if blend_mm is not None else -1.0)))

        code = -1
        for what, kw in attempts:
            fn = kw.pop("fn")
            kw = {k: val for k, val in kw.items() if val is not None}
            try:
                code = self._call(fn, **kw)
            except TypeError as e:
                print(f"     !! {what} could not be called: {e}")
                continue
            if code == 0:
                return
            print(f"     !! {what} returned {code}"
                  + (f": {FAIRINO_ERRORS[code]}" if code in FAIRINO_ERRORS else ""))
            try:
                rpc.ResetAllError()
            except Exception:
                pass
            time.sleep(0.3)

        meaning = FAIRINO_ERRORS.get(code, "")
        raise Refused(f"the controller refused '{label or kind}'"
                      + (f" -- {meaning}" if meaning else ""),
                      code=code if isinstance(code, int) else None)

    def stop(self) -> None:
        """
        A second, plain connection: the SDK's own is blocked inside the move
        being interrupted. This is StopMotion, not an E-stop.
        """
        try:
            xmlrpc.client.ServerProxy(
                f"http://{self.ip}:{self.stop_port}").StopMotion()
            if not self.quiet:
                print("\n  StopMotion sent")
        except Exception as e:
            raise CellError(f"StopMotion failed: {e} -- use the E-stop") from e

    def go_home(self, vel=None) -> None:
        if self.home:
            self.move(joints=self.home, kind="joint",
                      vel=vel if vel is not None else 10.0, label="home")

    def set_speed(self, percent: float) -> None:
        try:
            self._rpc().SetSpeed(float(percent))
        except Exception as e:
            print(f"  !! global speed could not be set: {e}")

    # -- I/O, for the sniffer's trigger and ready lines --------------------
    def read_input(self, index: int) -> int:
        return self._value(self._rpc().GetDI(int(index), 0), 0)

    def write_output(self, index: int, value) -> None:
        self._rpc().SetDO(int(index), int(bool(value)))
