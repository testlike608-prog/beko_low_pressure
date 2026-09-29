"""
The application.

    python main.py              one cycle, for real
    python main.py --dry        plan everything, move nothing
    python main.py --loop       cycle after cycle until Ctrl+C
    python main.py --expect 6   fail unless exactly six welds are found

The shape of this file is the point: everything comes up, one cycle runs, and
everything goes down -- and the going-down happens in a `finally` so it happens
after a crash, after Ctrl+C, and after a leak tester that never answered.
"""

from __future__ import annotations

import argparse
import sys
import time

import camera
import cell
import robot
import sniffer
import vision
from camera.hub import Hub
from camera_hub import CameraHub
from core import CellError

# ============================================================== the camera
# THIS file picks the camera, not settings.py.
#
#   camera_hub.py   (project root)  the drivers: RealSense, OpenCV, UseePlus
#   camera/hub.py                   the adapter that makes one of them speak
#                                   this project's Frame / Lens interface
#
# Change "backend" and nothing else in the project moves -- cell.py still calls
# camera.read() and neither it nor the model ever learns which camera this is.
#
# Extra keys are handed straight to the camera_hub driver, so a knob that
# exists over there works here with no edit in between.
CAMERA = {
    "backend": "realsense",     # realsense | opencv | useeplus | fake | webcam
    "camera_index": 0,
    # The D435I on this cell is on USB 2.1, so 720p is only offered at 6 fps.
    # If the device refuses this, camera_hub falls back and says so in the log.
    "frame_width": 1280,
    "frame_height": 720,
    "fps": 6,
    # "serial": "123456789012",         # only when two cameras are plugged in
}

# Run with no hardware by replacing the block above with one of these:
#   CAMERA = {"backend": "fake"}                       a synthetic scene
#   CAMERA = {"backend": "fake", "folder": "captures"} replay real captures
#   CAMERA = {"backend": "useeplus", "assume_depth_m": 0.25, "fov_deg": 70}


def build_camera():
    """
    The one place the camera is chosen -- check.py imports this too, so the
    test ladder opens exactly the camera a real run opens.

    A camera_hub backend is built right here, explicitly. Anything else falls
    through to the camera package, which is what keeps "fake" and "webcam"
    working unchanged.
    """
    backend = CAMERA.get("backend", "fake")
    if backend in Hub.BACKENDS:
        return Hub(**CAMERA)
    # Not a camera_hub driver, so the camera package has it. The block above is
    # written for camera_hub, and swapping one word should not then fail on a
    # leftover key -- so drop the ones this driver has never heard of and say
    # which, the way the simulator arm does with ip and tool.
    rest = {k: v for k, v in CAMERA.items() if k != "backend"}
    keep, dropped = _settings_this_driver_takes(backend, rest)
    if dropped:
        print(f"  camera: {backend} ignoring {', '.join(sorted(dropped))} "
              f"-- camera_hub settings that mean nothing here")
    return camera.build({"kind": backend, **keep})


def _settings_this_driver_takes(kind: str, config: dict) -> tuple[dict, set]:
    """(accepted, ignored) for a camera-package driver. Unknown kind -> let
    camera.build() be the one to complain, it words it better."""
    import importlib
    import inspect

    if kind not in camera.REGISTRY:
        return config, set()
    module_name, class_name = camera.REGISTRY[kind]
    try:
        cls = getattr(importlib.import_module(module_name), class_name)
        params = inspect.signature(cls.__init__).parameters
    except Exception:
        return config, set()
    if any(p.kind is inspect.Parameter.VAR_KEYWORD for p in params.values()):
        return config, set()
    keep = {k: v for k, v in config.items() if k in params}
    return keep, set(config) - set(keep)


def camera_hub_drivers() -> list[str]:
    """
    Which interfaces this copy of camera_hub.py actually carries.

    Worth printing: camera_hub.py is copied between projects by hand, so an
    older copy without RealSense is a real way to lose an afternoon.
    """
    return [name for name in sorted(Hub.BACKENDS.values())
            if hasattr(CameraHub, name)]


def start_all(need_robot=True, need_camera=True) -> None:
    """
    Bring the cell up. Order matters:

    the robot FIRST, because it is the one that announces itself, asks about
    AUTOMATIC mode, and can refuse -- better to find that out before a camera
    is holding a USB device and a model is holding half a gigabyte of GPU.
    """
    if need_robot:
        robot.start()
    if need_camera:
        camera.adopt(build_camera())
    vision.start()
    sniffer.start()


def stop_all() -> None:
    """
    Take the cell down. Reverse order, and every step independent: one that
    fails must not prevent the next. robot.stop() goes last because it is the
    one that matters most and the one most likely to have something to say.
    """
    sniffer.stop()
    vision.stop()
    camera.stop()
    robot.stop()


def describe() -> None:
    import settings
    print("\n  cell:")
    print(f"    robot    {robot.describe()}")
    cam = (camera.describe() if camera.is_running()
           else f"{build_camera().describe()} (not open)")
    print(f"    camera   {cam}")
    print(f"    camera_hub drivers: {', '.join(camera_hub_drivers()) or 'NONE'}")
    print(f"    vision   {vision.describe()}")
    print(f"    sniffer  {sniffer.describe()}")
    try:
        from core import HandEye
        print(f"    {HandEye(settings.HANDEYE).describe()}")
    except CellError as e:
        print(f"    !! hand-eye: {e}")
    print()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="weld-point leak inspection cell")
    parser.add_argument("--dry", action="store_true",
                        help="plan every pose, move nothing")
    parser.add_argument("--loop", action="store_true",
                        help="repeat until Ctrl+C")
    parser.add_argument("--expect", type=int, default=None,
                        help="fail unless exactly this many welds are found")
    parser.add_argument("--describe", action="store_true",
                        help="print the configured cell and exit")
    args = parser.parse_args(argv)

    if args.describe:
        describe()
        return 0

    try:
        start_all()
        describe()

        while True:
            results = cell.run(dry=args.dry, expected_points=args.expect)
            cell.report(results)
            if not args.loop:
                return 0 if all(r.ok for r in results) or args.dry else 1
            print("  next cycle in 2 s -- Ctrl+C to stop")
            time.sleep(2.0)

    except KeyboardInterrupt:
        print("\n\n  interrupted")
        robot.halt()
        return 130
    except CellError as e:
        # A sentence, not a stack. Everything in this project that the operator
        # can act on is raised as a CellError precisely so it lands here.
        print(f"\n  !! {e}\n")
        return 1
    finally:
        stop_all()


if __name__ == "__main__":
    sys.exit(main())
