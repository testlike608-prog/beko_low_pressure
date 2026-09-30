"""
cobot_kit -- the 3D-point-following-cobot cell, as modules app.py can use.

    from cobot_kit import Robot, Camera, AiModel, Locator
    from cobot_kit import cycle

    robot, camera = Robot(), Camera()
    ai, locator = AiModel("weld.pt"), Locator()
    robot.start(); camera.start(); ai.start()

    shots  = cycle.capture(robot, camera)          # capture positions + pictures
    shots  = cycle.detect(shots, ai)               # AI -> pixels
    points = cycle.locate(shots, locator)          # pixels -> base x, y, z
    # visiting the points = app.py's _robot_cycle (move / output / wait input)

    ai.stop(); camera.stop(); robot.stop()

Test with no hardware:   python -m cobot_kit.selftest
Values of this cell:     cobot_kit/settings.py
"""

from . import cycle, settings
from .ai import AiModel, ClickPicker, FixedPoints, FunctionPicker, draw
from .camera import Camera, Frame, Lens, Shot
from .cycle import approach_pose, point_pose
from .datatypes import CellError, Pixel, Point, Pose, Refused, Unreachable
from .locate import Locator
from .robot import Robot

__all__ = [
    "Robot", "Camera", "Frame", "Lens", "Shot",
    "AiModel", "ClickPicker", "FunctionPicker", "FixedPoints", "draw",
    "Locator", "point_pose", "approach_pose", "cycle", "settings",
    "Pose", "Pixel", "Point", "CellError", "Unreachable", "Refused",
]
