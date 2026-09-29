"""
The test ladder. Climb it in order; stop at the first rung that fails.

    python check.py                everything that needs no hardware
    python check.py 1              imports, settings, hand-eye maths
    python check.py 2              open the camera, grab a frame, save it
    python check.py 3              connect to the robot -- NOTHING MOVES
    python check.py 4              a small, slow, deliberate jog          [MOVES]
    python check.py 5              click a weld, see where the arm thinks it is
    python check.py 6              a full cycle, planned, nothing moving
    python check.py 7              a full cycle, for real                 [MOVES]

Each rung adds exactly one thing that can be wrong. That is the entire value:
when rung 5 gives a number 40 mm out, rungs 1-4 have already proved it is not
the maths, not the camera, not the connection and not the motion -- so it is
the calibration or the pendant's tool, and there is nowhere else to look.

Rungs 4 and 7 move the arm. They say so, and they ask first.
"""

from __future__ import annotations

import sys
import time

RUNGS = {}


def rung(number, title, moves=False):
    def wrap(fn):
        fn.number, fn.title, fn.moves = number, title, moves
        RUNGS[number] = fn
        return fn
    return wrap


def ok(message):
    print(f"    ok    {message}")


def bad(message):
    print(f"    !!    {message}")


def ask(question) -> bool:
    print(f"\n  *** {question}")
    try:
        return input("  *** type YES to continue: ").strip().upper() == "YES"
    except (EOFError, KeyboardInterrupt):
        return False


# =========================================================== 1. no hardware
@rung(1, "imports, settings and the hand-eye maths -- no hardware")
def check_basics() -> bool:
    import numpy as np

    import camera
    import robot
    import sniffer
    import vision
    from core import (BASE, CAMERA, Batch, HandEye, Point, Pose, invert,
                      matrix_to_pose, pose_to_matrix)

    ok(f"core, camera, robot, vision, sniffer all import")
    ok(f"camera kinds: {', '.join(camera.kinds())}")
    ok(f"robot kinds:  {', '.join(robot.kinds())}")
    ok(f"vision kinds: {', '.join(vision.kinds())}")

    import settings
    for name in ("CAMERA", "ROBOT", "VISION", "HANDEYE"):
        if not hasattr(settings, name):
            bad(f"settings.py has no {name}")
            return False
    ok("settings.py has every block")

    # The drivers are built but not opened -- this catches a typo in a settings
    # key today rather than in front of the arm tomorrow.
    try:
        import main
        main.build_camera()          # main.py owns the camera, so check main.py
        robot.build()
        vision.build()
        sniffer.build()
        ok("every configured driver accepts its settings")
    except Exception as e:
        bad(f"a driver rejected its settings: {e}")
        return False

    # A pose through the matrix and back must come out unchanged. If this ever
    # fails, nothing downstream is worth debugging.
    p = Pose(734.0, -426.0, -283.0, 178.0, -3.0, 175.0)
    back = matrix_to_pose(pose_to_matrix(p))
    if max(abs(a - b) for a, b in zip(p.list(), back.list())) > 1e-6:
        bad("pose -> matrix -> pose does not round-trip")
        return False
    ok("the rotation convention round-trips exactly")

    T = pose_to_matrix(p)
    if not np.allclose(invert(T) @ T, np.eye(4), atol=1e-9):
        bad("invert() is not an inverse")
        return False
    ok("rigid-transform inversion is exact")

    try:
        he = HandEye(settings.HANDEYE)
    except Exception as e:
        bad(f"{e}")
        return False
    ok(he.describe())
    for warning in he.warnings():
        bad(warning)

    # The transform is exercised with numbers only -- no camera, no robot.
    flange = Pose(600.0, -300.0, 100.0, 180.0, 0.0, 175.0)
    batch = Batch(points=[Point(0.0, 0.0, 450.0, CAMERA, "test")], flange=flange)
    moved = he.to_base(batch)[0]
    if moved.frame != BASE:
        bad("the hand-eye transform did not land in the base frame")
        return False
    ok(f"a point 450 mm in front of the lens lands at "
       f"({moved.x:.0f}, {moved.y:.0f}, {moved.z:.0f}) in the base frame")

    back = he.to_camera(moved, flange)
    if abs(back.z - 450.0) > 0.01:
        bad("base -> camera does not undo camera -> base")
        return False
    ok("the transform is reversible")
    return True


# ============================================================== 2. camera
@rung(2, "open the camera, grab a frame, save it")
def check_camera() -> bool:
    import camera

    import main

    try:
        cam = camera.adopt(main.build_camera())
    except Exception as e:
        bad(f"{e}")
        return False

    try:
        ok(cam.describe())
        ok(f"lens: {cam.lens}")

        t = time.time()
        frame = camera.read()
        ok(f"a frame arrived in {time.time() - t:.2f} s, "
           f"{frame.size[0]}x{frame.size[1]}")

        if frame.has_depth:
            coverage = frame.depth_coverage()
            middle = cam.depth_at(frame, frame.size[0] / 2, frame.size[1] / 2)
            ok(f"depth coverage {coverage * 100:.0f} %")
            if middle:
                ok(f"the centre pixel is {middle * 1000:.0f} mm away")
            else:
                bad("the centre pixel has no depth -- shiny, too close, or in "
                    "shadow. Move the camera back a little and look again.")
            if coverage < 0.4:
                bad("under 40 % of the image has depth. That is normal on bare "
                    "metal and a problem for weld points: more light, less "
                    "specular glare, or a shorter working distance.")
        else:
            bad("this camera has no depth. Set assume_depth_m in settings.py "
                "or the cell will produce no points.")

        path = camera.save(frame, "captures", "check")
        ok(f"saved {path} -- replay it later by setting main.py's "
           f"CAMERA = {{'backend': 'fake', 'folder': 'captures'}}")
        return True
    finally:
        camera.stop()


# =============================================================== 3. robot
@rung(3, "connect to the robot and read it -- NOTHING MOVES")
def check_robot() -> bool:
    import robot

    try:
        # home=False is the whole point of this rung: it comes up, checks the
        # frames, and stands still.
        arm = robot.start(home=False)
    except Exception as e:
        bad(f"{e}")
        return False

    try:
        ok(arm.describe())
        ok(f"tool   {robot.pose()}")
        ok(f"flange {robot.flange()}")
        ok("joints " + ", ".join(f"{j:.1f}" for j in robot.joints()))

        gap = robot.pose().distance_to(robot.flange())
        ok(f"the tip is {gap:.1f} mm from the flange")

        limits = arm.joint_limits()
        if limits:
            ok("soft limits " + " ".join(f"[{a:.0f},{b:.0f}]" for a, b in limits))
        else:
            bad("the controller would not report its joint limits -- an "
                "out-of-range target will come back as a bare error code")

        problems = arm.ready()
        if problems:
            for p in problems:
                bad(p)
        else:
            ok("the controller reports nothing blocking motion")

        problems = arm.verify()
        if problems:
            for p in problems:
                bad(p)
            bad("FIX THIS BEFORE RUNG 4. Frame mismatches are the single most "
                "expensive mistake in this cell.")
            return False
        ok("the controller solves in the same frames this project commands in")
        return True
    finally:
        robot.stop()


# ============================================================ 4. a small jog
@rung(4, "a small, slow, deliberate jog", moves=True)
def check_jog() -> bool:
    import robot
    import settings
    from core import Pose

    distance = 30.0
    if not ask(f"This moves the arm {distance:.0f} mm up and back, slowly. "
               f"Is the cell clear?"):
        print("    skipped")
        return True

    try:
        robot.start(home=False)
    except Exception as e:
        bad(f"{e}")
        return False

    try:
        here = robot.pose()
        ok(f"starting at {here}")

        up = here.shifted(dz=distance)
        if not robot.can_reach(up):
            bad(f"{up} has no joint solution -- try this rung from a different "
                f"starting posture")
            return False

        robot.go(up, vel=5.0, label="jog up")
        time.sleep(0.5)
        reached = robot.pose()
        error = abs(reached.z - up.z)
        ok(f"reached {reached}")
        if error > 2.0:
            bad(f"asked for z {up.z:.1f}, got {reached.z:.1f} -- {error:.1f} mm "
                f"short. Check the pendant's tool and workpiece frames.")
        else:
            ok(f"within {error:.2f} mm of the commanded pose")

        robot.go(here, vel=5.0, label="jog back")
        ok(f"back at {robot.pose()}")
        return error <= 2.0
    finally:
        robot.stop()


# ======================================================== 5. camera <-> robot
@rung(5, "click a weld and see where the arm thinks it is")
def check_handeye() -> bool:
    """
    The one rung that tests the JOIN between the two halves.

    Nothing moves. You click a feature you can measure -- a corner, a mark on
    the table -- and the cell prints where that pixel lands in the robot base
    frame. Then you jog the arm there by hand on the pendant and read the
    difference. That difference IS the cell's accuracy, and no amount of
    reasoning replaces measuring it once.
    """
    import camera
    import robot
    import settings
    import vision
    from core import HandEye

    try:
        import main
        robot.start(home=False)
        camera.adopt(main.build_camera())
        vision.start({"kind": "clicks"})
    except Exception as e:
        bad(f"{e}")
        robot.stop()
        camera.stop()
        return False

    try:
        he = HandEye(settings.HANDEYE)
        ok(he.describe())

        flange = robot.flange()
        frame = camera.read()
        after = robot.flange()
        if flange.distance_to(after) > 1.0:
            bad("the arm moved during the capture -- let it settle and retry")
            return False
        ok(f"flange at capture: {flange}")

        print("\n  click one feature you can also touch with the tip, "
              "then press ENTER")
        batch = vision.look(frame, flange)
        if not len(batch):
            bad("nothing was clicked")
            return False

        points = he.to_base(batch)
        print()
        for p in points:
            uv = p.meta.get("uv", (0, 0))
            depth = p.meta.get("depth_m", 0) * 1000
            ok(f"pixel ({uv[0]:.0f}, {uv[1]:.0f}) at {depth:.0f} mm  ->  "
               f"base ({p.x:.1f}, {p.y:.1f}, {p.z:.1f})")

        print("\n  Now jog the TIP to that feature on the pendant and compare "
              "its readout\n  with the numbers above. The difference is this "
              "cell's real accuracy.")
        print(f"  The tip is currently at {robot.pose()}\n")
        return True
    finally:
        vision.stop()
        camera.stop()
        robot.stop()


# ============================================================== 6. dry cycle
@rung(6, "a full cycle, planned, nothing moving")
def check_dry() -> bool:
    import main

    return main.main(["--dry"]) == 0


# ============================================================= 7. real cycle
@rung(7, "a full cycle, for real", moves=True)
def check_real() -> bool:
    import main

    if not ask("This runs the whole cycle and the arm will visit every "
               "detected point. Is the cell clear?"):
        print("    skipped")
        return True
    return main.main([]) == 0


# ================================================================== driver
def run(numbers) -> int:
    failed = []
    for n in numbers:
        fn = RUNGS[n]
        flag = "   [MOVES THE ARM]" if fn.moves else ""
        print(f"\n{'=' * 68}\n  {n}. {fn.title}{flag}\n{'=' * 68}")
        try:
            passed = bool(fn())
        except KeyboardInterrupt:
            print("\n  interrupted")
            return 130
        except Exception as e:
            bad(f"{type(e).__name__}: {e}")
            passed = False
        if not passed:
            failed.append(n)
            print(f"\n  rung {n} FAILED -- fix this before climbing further")
            break

    print(f"\n{'=' * 68}")
    if failed:
        print(f"  failed at rung {failed[0]}: {RUNGS[failed[0]].title}")
        return 1
    print(f"  rungs {', '.join(str(n) for n in numbers)} all passed")
    return 0


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    if args:
        chosen = [int(a) for a in args]
    else:
        # No argument: everything that needs no hardware and moves nothing.
        chosen = [1]
        print("  running the rungs that need no hardware.\n"
              "  Add a number to go further: python check.py 2\n"
              "  Rungs: " + "  ".join(f"{n}={f.title.split(' --')[0][:28]}"
                                      for n, f in sorted(RUNGS.items())))
    sys.exit(run(chosen))
