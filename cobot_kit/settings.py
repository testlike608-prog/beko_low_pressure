"""
cobot_kit settings -- the values of THIS cell, in one place.

Every class in the kit reads its defaults from here, and every one of them
also takes the same names as keyword arguments, so app.py can override any
single value without editing this file:

    Robot()                        everything below
    Robot(ip="192.168.58.2")       same, different controller
    Robot(kind="simulator")        no hardware at all
"""

# ================================================================ the robot
ROBOT = {
    "kind": "fairino",              # "fairino" | "simulator"
    "ip": "192.168.57.2",           # NOT the Fairino default .58.2
    "tool": 6,                      # MUST match the pendant's active tool
    "user": 0,                      # MUST match the pendant's workpiece frame
    "tip_length_mm": 157.0,         # the sniffer tip; checked by verify()
    "home": [0.0, -20.0, -90.0, -70.0, 90.0, 0.0],   # joints, degrees
    "auto_mode": True,              # False = never switch to AUTOMATIC itself
    "speed": 30.0,                  # the controller's global override, %
}
HOME_ON_START = True
HOME_ON_STOP = True
HOME_VELOCITY = 10.0
VERIFY_ON_START = True              # check the pendant tool BEFORE any move


# =============================================================== the camera
# backend: "realsense" | "opencv" | "useeplus" (camera_hub.py) | "fake"
CAMERA = {
    "backend": "realsense",
    "camera_index": 0,
    # The D435I on this cell is on USB 2.1 -> 720p only at 6 fps.
    "frame_width": 1280,
    "frame_height": 720,
    "fps": 6,
}
CAPTURE_FOLDER = "capture"          # beside app.py -- the folder you made


# ============================================================ the hand-eye
HANDEYE = "handeye.json"            # flange <- camera, eye-in-hand


# ================================================================ the model
AI = {
    "weights": "weld.pt",           # YOLO weights (.pt), beside app.py or here
    "task": "detect",               # "detect" | "segment" | "pose"
    "min_confidence": 0.5,
}


# =============================================================== the cycle
# Where the arm stands to take the pictures. Each entry is either
#   {"pose":   [x, y, z, rx, ry, rz]}     base frame, mm / deg
#   {"joints": [j1, j2, j3, j4, j5, j6]}  degrees -- safer, no IK involved
CAPTURE_POSITIONS = [
    # {"joints": [0.0, -20.0, -90.0, -70.0, 90.0, 0.0]},
]
SETTLE_S = 0.5                      # wait after reaching a capture position

VELOCITY = 20.0                     # default velocity of move_to / move_joints, %
# Approach distance, output / input numbers etc. live in app.py's _robot_cycle.
