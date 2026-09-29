"""
The cycle: capture -> detect -> hand-eye -> visit each point -> sniff -> report.

This is the only file that knows the ORDER of things. Every other module knows
how to do one job and nothing about when it happens, which is what lets you
swap a camera, a model or an arm without reading this file at all.

Two entry points:

    plan()      work out every pose without moving anything.
                This is `--dry`, and it is where most mistakes are caught.

    run()       the same plan, executed.

plan() is deliberately the bigger half. Everything that can be decided without
the arm moving is decided before it moves: reachability, the approach line, the
point count against what the recipe expects, the workspace box. A cycle that
fails should fail at the plan, standing still.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

import camera
import robot
import sniffer
import vision
from core import (BASE, Batch, CellError, HandEye, Point, Pose, Refused, Result,
                  Unreachable, pose_facing, tool_axis)


@dataclass
class Target:
    """One planned visit: where to stand off, where to go, what it is."""

    index: int
    point: Point
    pose: Pose
    approach: Pose | None = None
    reachable: bool = True
    why_not: str = ""
    meta: dict = field(default_factory=dict)

    @property
    def label(self) -> str:
        return self.point.label or f"point{self.index}"

    def line(self) -> str:
        mark = "  " if self.reachable else "!!"
        return (f"  {mark} {self.index:>2}. {self.label:<16} {self.pose}"
                + (f"   {self.why_not}" if self.why_not else ""))


# --------------------------------------------------------------------- look
def capture(save=True) -> tuple:
    """
    One look: move to the capture pose if there is one, grab a frame, read the
    flange pose AROUND the capture, detect.

    Returns (frame, batch). The flange pose is read immediately before and
    checked immediately after: if the arm drifted more than a millimetre
    between them, the capture happened while something was still moving and the
    batch would be transformed with a pose that was never true. Better to say
    so than to produce points that are quietly a centimetre off.
    """
    import settings

    if settings.CAPTURE_POSE:
        robot.go(Pose.of(settings.CAPTURE_POSE), vel=settings.VELOCITY,
                 label="capture pose")
        time.sleep(0.3)

    before = robot.flange()
    frame = camera.read()
    after = robot.flange()

    drift = before.distance_to(after)
    if drift > 1.0:
        raise CellError(
            f"the arm moved {drift:.1f} mm during the capture. Every point "
            f"from this frame would be wrong by about that much. Let the arm "
            f"settle (raise DWELL_S) and look again.")

    batch = vision.look(frame, before)

    if save and getattr(settings, "SAVE_CAPTURES", None):
        try:
            path = camera.save(frame, settings.SAVE_CAPTURES)
            batch.meta["capture"] = str(path)
        except Exception as e:
            print(f"  !! the frame could not be saved: {e}")

    return frame, batch


# --------------------------------------------------------------------- plan
def plan(batch: Batch, from_pose: Pose | None = None) -> list[Target]:
    """
    Camera-frame points -> base-frame poses the arm can be commanded to.

    The orientation is the question this function answers. A point in space
    does not say which way the sniffer should face, so:

        the point carries a normal    come in ALONG it (a service port)
        it does not                   keep the orientation the arm has now,
                                      which is the one the operator chose when
                                      they set up the capture pose

    The second rule is the boring one and it is right far more often than a
    clever default would be.
    """
    import settings

    handeye = HandEye(settings.HANDEYE)
    for warning in handeye.warnings():
        print(f"  !! {warning}")

    points = handeye.to_base(batch)
    if from_pose is None:
        from_pose = robot.pose() if robot.is_running() else Pose(0, 0, 0, 180, 0, 0)

    targets: list[Target] = []
    for i, point in enumerate(points, 1):
        if point.normal is not None:
            # The normal points from the target outwards; the tool's z must
            # point the other way, INTO the part.
            pose = pose_facing(point.xyz, -np.asarray(point.normal, dtype=float))
        else:
            pose = from_pose.at(point.xyz)

        axis = tool_axis(pose)                       # out of the tip

        # Back off along the tool's own axis. Not along z-up: the tool is
        # rarely vertical, and backing off the wrong way is how a sniffer meets
        # the part it was avoiding.
        if settings.CLEARANCE_MM:
            pose = pose.at(np.asarray(pose.xyz) - axis * settings.CLEARANCE_MM)

        approach = None
        if settings.APPROACH_MM:
            approach = pose.at(np.asarray(pose.xyz) - axis * settings.APPROACH_MM)

        target = Target(i, point, pose, approach)

        if pose.reach > settings.MAX_REACH_MM:
            target.reachable = False
            target.why_not = (f"{pose.reach:.0f} mm from the base, past the "
                              f"{settings.MAX_REACH_MM:.0f} mm limit in settings")
        elif robot.is_running() and not robot.can_reach(pose):
            target.reachable = False
            target.why_not = "the controller has no joint solution for this pose"

        targets.append(target)

    return targets


def check_count(targets, expected=None) -> None:
    """
    The recipe says how many welds this SKU has. A model that found five on a
    six-weld fridge has missed one, and the missed one is exactly the one worth
    testing -- so this is a hard stop, not a warning.
    """
    import settings

    n = len(targets)
    if n < settings.MIN_POINTS:
        raise CellError(f"only {n} welding points were found; settings.py "
                        f"requires at least {settings.MIN_POINTS}")
    if n > settings.MAX_POINTS:
        raise CellError(f"{n} welding points were found, more than the "
                        f"{settings.MAX_POINTS} this cell allows -- the model "
                        f"is detecting things that are not welds")
    if expected is not None and n != expected:
        raise CellError(f"the recipe expects {expected} welding points and "
                        f"{n} were found. Do not leak-test a partial set.")


# ---------------------------------------------------------------------- run
def visit(target: Target) -> Result:
    """Go to one target and test it. Never raises for a bad point."""
    import settings

    started = time.time()
    result = Result(target.index, target.label, target.point, target.pose)

    if not target.reachable:
        result.note = target.why_not
        result.seconds = time.time() - started
        return result

    try:
        if target.approach is not None:
            robot.go(target.approach, vel=settings.VELOCITY,
                     label=f"{target.label} approach")
        robot.go(target.pose, vel=settings.APPROACH_VELOCITY,
                 kind="linear" if target.approach is not None else "joint",
                 label=target.label)
        result.reached = True

        if settings.DWELL_S:
            time.sleep(settings.DWELL_S)

        verdict, value = sniffer.test(target.label)
        result.leak, result.value = verdict, value

        if settings.RETREAT and target.approach is not None:
            robot.go(target.approach, vel=settings.APPROACH_VELOCITY,
                     kind="linear", label=f"{target.label} retreat")

    except Unreachable as e:
        result.note = f"unreachable: {e}"
    except Refused as e:
        result.note = f"refused: {e}"
    except CellError as e:
        result.note = str(e)

    result.seconds = time.time() - started
    return result


def run(dry: bool = False, expected_points: int | None = None) -> list[Result]:
    """
    One full cycle, assuming start() has already been called on everything.

    main.py is the version that also starts and stops things. This one exists
    separately so a cycle can be run twice without bringing the arm down in
    between -- which is what a production loop does.
    """
    import settings

    frame, batch = capture()
    print(f"\n  {len(batch)} welding point(s) from {batch.meta.get('detector')}")

    targets = plan(batch)
    check_count(targets, expected_points)

    print("\n  plan:")
    for t in targets:
        print(t.line())

    blocked = [t for t in targets if not t.reachable]
    if blocked:
        print(f"\n  !! {len(blocked)} of {len(targets)} points cannot be "
              f"reached and will be skipped")

    if dry:
        print("\n  dry run -- nothing moved\n")
        return [Result(t.index, t.label, t.point, t.pose,
                       reached=False, note=t.why_not or "dry run")
                for t in targets]

    results = []
    for target in targets:
        print(f"\n  -> {target.index}. {target.label}")
        result = visit(target)
        print(result.line())
        results.append(result)

    return results


# ------------------------------------------------------------------- report
def report(results, path=None) -> None:
    """Print the summary, and append one JSON line per run to the results file."""
    import settings

    print("\n  " + "-" * 62)
    for r in results:
        print(r.line())

    passed = sum(1 for r in results if r.leak == "pass")
    failed = sum(1 for r in results if r.leak == "fail")
    missed = sum(1 for r in results if not r.reached)
    print(f"\n  {len(results)} point(s): {passed} pass, {failed} fail, "
          f"{missed} not reached")
    print("  " + "-" * 62 + "\n")

    path = path or getattr(settings, "RESULTS_FILE", None)
    if not path:
        return
    try:
        record = {"when": time.strftime("%Y-%m-%dT%H:%M:%S"),
                  "passed": passed, "failed": failed, "missed": missed,
                  "results": [r.dict() for r in results]}
        with Path(path).open("a", encoding="utf-8") as f:
            f.write(json.dumps(record) + "\n")
        print(f"  appended to {path}")
    except Exception as e:
        print(f"  !! the results could not be written: {e}")
