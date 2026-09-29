"""
The only file you edit day to day.

Plain Python, so a value can be a number, a list, or a small expression, and
your editor tells you about a typo before the arm does. Every block below is a
dict with a "kind" and whatever that kind takes -- the same shape for the
camera, the robot, the model and the leak tester.

Change one "kind" and the whole project follows. Nothing else knows the
difference.
"""

# =============================================================== the camera
#
# main.py OWNS THE CAMERA. It builds it from camera_hub.py -- look for the
# CAMERA block at the top of main.py, and change it there.
#
# The block below is only what runs when a camera is started WITHOUT going
# through main.py: the test suite, and camera.build() with no config. It is
# deliberately the fake one, so a stray run can never wake real hardware.
CAMERA = {"kind": "fake"}

# ================================================================ the robot
# kinds: "fairino" | "simulator"                 (robot/__init__.py REGISTRY)
ROBOT = {
    "kind": "fairino",
    "ip": "192.168.57.2",           # NOT the Fairino default .58.2
    "tool": 6,                      # MUST match the pendant's active tool
    "user": 0,                      # MUST match the pendant's workpiece frame
    "tip_length_mm": 157.0,         # the sniffer tip; checked by verify()
    "home": [0.0, -20.0, -90.0, -70.0, 90.0, 0.0],
    "auto_mode": True,              # False = never switch to AUTOMATIC itself
    "speed": 30.0,
}

# ROBOT = {"kind": "simulator"}     no hardware, full cycle


# =============================================================== the model
# kinds: "yolo" | "clicks" | "fixed" | "file"    (vision/__init__.py REGISTRY)
#
# Until the weld-point model is trained, "clicks" is the working setup: you
# click the welds and the arm visits them.
VISION = {
    "kind": "clicks",
    # "expect": 6,                  # stop after this many clicks
}

# Once the model exists:
#   VISION = {"kind": "yolo", "weights": "weld.pt", "min_confidence": 0.5}
#
# For the service port, whose AXIS matters and not just its position, train a
# pose model marking the mouth and the far end, then:
#   VISION = {"kind": "yolo", "weights": "weld-pose.pt", "task": "pose",
#             "axis_from_keypoints": (0, 1)}
#
# With no hardware and no model:
#   VISION = {"kind": "fixed", "count": 4}
#   VISION = {"kind": "file", "path": "points.csv"}


# =========================================================== the leak tester
# kinds: "fake" | "digital"                    (sniffer/__init__.py REGISTRY)
SNIFFER = {
    "kind": "fake",
    "seconds": 0.3,
}

# When the Inficon is wired through the controller's I/O:
#   SNIFFER = {"kind": "digital", "trigger_out": 0, "ready_in": 0,
#              "pass_in": 1, "fail_in": 2}


# ============================================================ the hand-eye
# Camera position in the flange frame. Recalibrate only when the bracket moves.
HANDEYE = "handeye.json"


# =============================================================== the cycle
CAPTURE_POSE = None          # None = look from wherever the arm is standing.
                             # Otherwise six numbers, the base-frame pose the
                             # arm moves to before every capture:
                             # CAPTURE_POSE = [734, -426, -100, 180, 0, 175]

APPROACH_MM = 60.0           # stand off this far along the tool axis first
CLEARANCE_MM = 0.0           # stop this far SHORT of the weld. The sniffer
                             # does not touch: leave a gap, or set a negative
                             # number to insert into the service port.
DWELL_S = 1.0                # hold still before triggering the sniffer -- the
                             # arm is still ringing for a moment after a move
RETREAT = True               # back out along the approach line after each point

VELOCITY = 20.0              # per-move velocity, percent
APPROACH_VELOCITY = 8.0      # slower for the last leg, near the part
SPEED_PERCENT = 30.0         # the controller's global override

MAX_REACH_MM = 900.0         # refuse anything further out than this, in
                             # software, before the controller has to
MIN_POINTS = 1               # fewer detections than this fails the cycle
MAX_POINTS = 24              # more than this means the model is hallucinating

HOME_ON_START = True
HOME_ON_STOP = True
HOME_VELOCITY = 10.0


# =============================================================== reporting
SAVE_CAPTURES = "captures"   # every frame is saved here; None to switch off
RESULTS_FILE = "results.jsonl"
