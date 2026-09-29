"""
robot.start() / robot.stop(), on the simulator -- plus the Fairino driver's
pure logic, tested without a controller.
"""

import ctypes
import threading
import time

import pytest

import robot
from core import CellError, Pose, Refused, Unreachable
from robot.fairino import Fairino
from robot.simulator import Simulator

SIM = {"kind": "simulator", "speed_factor": 0.0, "quiet": True}


@pytest.fixture(autouse=True)
def _down():
    # home=False on purpose: settings.HOME_ON_STOP is True for the real cell,
    # and a homing move at the simulator's real speed makes the suite crawl.
    robot.stop(home=False)
    yield
    robot.stop(home=False)


# --------------------------------------------------------------- start/stop
def test_start_brings_the_arm_up():
    arm = robot.start(SIM, home=False)
    assert robot.is_running()
    assert isinstance(arm, Simulator)
    assert isinstance(robot.pose(), Pose)


def test_start_twice_returns_the_same_arm():
    a = robot.start(SIM, home=False)
    b = robot.start(SIM, home=False)
    assert a is b


def test_stop_without_start_is_a_no_op():
    robot.stop()
    robot.stop()
    assert not robot.is_running()


def test_stop_twice_is_a_no_op():
    robot.start(SIM, home=False)
    robot.stop()
    robot.stop()
    assert not robot.is_running()


def test_start_homes_when_asked():
    arm = robot.start(SIM, home=True)
    assert any("home" in line for line in arm.log)


def test_start_can_skip_homing():
    arm = robot.start(SIM, home=False)
    assert not any("home" in line for line in arm.log)


def test_stop_never_raises_even_when_every_step_fails():
    """The contract that matters: stop() runs in a finally block, so a driver
    that throws on every call must still leave the module cleanly stopped."""

    class Awkward(Simulator):
        def stop(self):
            raise RuntimeError("no")

        def release(self):
            raise RuntimeError("no")

        def disconnect(self):
            raise RuntimeError("no")

        def go_home(self, vel=None):
            raise RuntimeError("no")

    robot._robot = Awkward(quiet=True)
    robot.stop(home=True)                    # must not raise
    assert not robot.is_running()


class Refuses(Simulator):
    """An arm that connects and then will not come up. Module level, because
    the registry imports drivers by name."""

    def prepare(self):
        raise CellError("the servos will not enable")


def test_a_failed_start_leaves_nothing_running():
    robot.REGISTRY["refuses"] = (__name__, "Refuses")
    try:
        with pytest.raises(CellError):
            robot.start({"kind": "refuses", "quiet": True}, home=False)
        assert not robot.is_running()
    finally:
        robot.REGISTRY.pop("refuses", None)


def test_using_the_arm_before_start_says_what_to_do():
    with pytest.raises(CellError) as e:
        robot.pose()
    assert "start" in str(e.value)


def test_an_unknown_kind_names_the_known_ones():
    with pytest.raises(CellError) as e:
        robot.build({"kind": "kuka"})
    assert "fairino" in str(e.value) and "simulator" in str(e.value)


# ------------------------------------------------------------------- motion
def test_go_moves_and_reports():
    robot.start(SIM, home=False)
    target = Pose(600, -300, 250, 180, 0, 175)
    robot.go(target, label="w1")
    assert robot.pose().distance_to(target) < 1e-6


def test_out_of_reach_is_refused_before_moving():
    robot.start(SIM, home=False)
    before = robot.pose()
    with pytest.raises(Unreachable):
        robot.go(Pose(5000, 0, 0, 180, 0, 0))
    assert robot.pose() == before


def test_can_reach_answers_without_moving():
    robot.start(SIM, home=False)
    before = robot.pose()
    assert robot.can_reach(Pose(600, -300, 250, 180, 0, 175))
    assert not robot.can_reach(Pose(5000, 0, 0, 180, 0, 0))
    assert robot.pose() == before


def test_can_reach_is_true_when_the_arm_has_no_ik():
    class NoIK(Simulator):
        has_ik = False

    robot._robot = NoIK(quiet=True)
    assert robot.can_reach(Pose(9999, 0, 0)) is True


def test_halt_interrupts_a_move_in_flight():
    arm = robot.start({"kind": "simulator", "speed_factor": 0.02, "quiet": True},
                      home=False)
    problem = []

    def move():
        try:
            robot.go(Pose(700, -200, 400, 180, 0, 175), vel=10, label="long")
        except Refused as e:
            problem.append(e)

    thread = threading.Thread(target=move)
    thread.start()
    time.sleep(0.15)
    robot.halt()
    thread.join(timeout=5)

    assert problem, "the move should have been refused when it was halted"
    assert robot.is_running(), "halt() must not disconnect"


# ------------------------------------------------------- the Fairino driver
def test_value_handles_both_sdk_return_shapes():
    assert Fairino._value(0) is None                     # a bare error code
    assert Fairino._value((0, [1, 2, 3])) == [1, 2, 3]   # (code, value)
    assert Fairino._value((7, [1, 2, 3])) is None        # a failed call


def test_call_drops_arguments_this_sdk_build_does_not_have():
    """blendR exists on some builds and not others. A driver that hard-codes it
    works on one factory PC and raises TypeError on the next."""
    def move(desc_pos, tool, user):
        return (desc_pos, tool, user)

    assert Fairino._call(move, desc_pos=[1], tool=6, user=0,
                         blendR=-1.0) == ([1], 6, 0)


def test_field_reads_a_ctypes_structure_instance():
    class State(ctypes.Structure):
        _fields_ = [("robot_mode", ctypes.c_int)]

    assert Fairino._field(State(robot_mode=0), "robot_mode") == (0, True)


def test_field_reads_through_a_pointer():
    class State(ctypes.Structure):
        _fields_ = [("robot_mode", ctypes.c_int)]

    s = State(robot_mode=1)
    assert Fairino._field(ctypes.pointer(s), "robot_mode") == (1, True)


def test_field_survives_the_structure_class_itself():
    """
    The real-hardware bug: on some SDK builds robot_state_pkg comes back as the
    ctypes Structure CLASS, so every attribute is a field descriptor and int()
    on it raises "not '_ctypes.CField'" -- which killed the run before the
    first move and says nothing about a robot.

    It must come back as not-read, NOT as a plausible value, because "I cannot
    see the mode" and "the mode is fine" need different reactions.
    """
    class State(ctypes.Structure):
        _fields_ = [("robot_mode", ctypes.c_int)]

    value, read = Fairino._field(State, "robot_mode")
    assert read is False
    assert value == -1


def test_field_on_a_missing_name_or_none():
    assert Fairino._field(None, "robot_mode") == (-1, False)
    assert Fairino._field(object(), "nope", default=5) == (5, False)


def test_the_fairino_driver_imports_without_the_sdk():
    arm = Fairino(ip="192.168.57.2", tool=6)
    assert arm.has_ik and arm.can_stop_mid_move and arm.has_io
    with pytest.raises(CellError):
        arm.joints()                       # not connected, and it says so


def test_the_driver_reports_its_capabilities_honestly():
    text = Fairino(ip="1.2.3.4").describe()
    assert "IK" in text and "linear" in text
