"""
The one transform that joins the camera half of this project to the robot half.

    point in CAMERA mm  ->  point in ROBOT BASE mm

Eye-in-hand, so the chain is:

    base <- flange          read from the controller at the instant of capture
    flange <- camera        the calibration in handeye.json (never changes
                            unless the bracket is touched)

    T_base_camera = T_base_flange @ T_flange_camera

The flange pose comes from the Batch, not from a fresh read. That is the whole
point of Batch carrying it: transforming with a pose read after the arm has
moved is a tens-of-millimetres error that looks exactly like a bad calibration
and sends people back to re-calibrate a camera that was fine.

Nothing here touches a camera or a robot, so it is fully testable with numbers.
"""

from __future__ import annotations

import json

import numpy as np

from .geometry import invert, move_points
from .types import BASE, CAMERA, Batch, CellError, Point, Pose, find_file

EYE_IN_HAND = "eye_in_hand"
EYE_TO_HAND = "eye_to_hand"


class HandEye:
    """
    A loaded calibration.

        he = HandEye("handeye.json")
        base_points = he.to_base(batch)

    `mode` decides what the stored matrix means:

        eye_in_hand    camera bolted to the flange   (this cell)
        eye_to_hand    camera bolted to the world; the flange pose is then
                       irrelevant and the matrix is base <- camera directly
    """

    def __init__(self, path="handeye.json"):
        self.path = find_file(path, "hand-eye calibration")
        data = json.loads(self.path.read_text(encoding="utf-8"))

        if "matrix_mm" not in data:
            raise CellError(f"{self.path} has no 'matrix_mm' -- this is not a "
                            f"hand-eye calibration file")

        self.mode = data.get("mode", EYE_IN_HAND)
        if self.mode not in (EYE_IN_HAND, EYE_TO_HAND):
            raise CellError(f"{self.path}: unknown mode {self.mode!r}")

        self.matrix = np.asarray(data["matrix_mm"], dtype=float)
        if self.matrix.shape != (4, 4):
            raise CellError(f"{self.path}: matrix_mm must be 4x4, "
                            f"got {self.matrix.shape}")

        self.rms_mm = float(data.get("rms_mm", 0.0))
        self.n_poses = int(data.get("n_poses", 0))
        self.camera_matrix = data.get("camera_matrix")
        self.board = data.get("board")

    # ------------------------------------------------------------------ use
    def base_from_camera(self, flange: Pose | None) -> np.ndarray:
        """The 4x4 that takes camera-frame points into the base frame."""
        if self.mode == EYE_TO_HAND:
            return self.matrix
        if flange is None:
            raise CellError(
                "an eye-in-hand calibration needs the flange pose from the "
                "moment of capture. The Batch carries it -- something built "
                "this batch without a robot attached.")
        from .geometry import pose_to_matrix
        return pose_to_matrix(flange) @ self.matrix

    def to_base(self, batch: Batch) -> list[Point]:
        """Camera-frame points in -> base-frame points out."""
        pts = list(batch.points) if isinstance(batch, Batch) else list(batch)
        if not pts:
            return []
        if pts[0].frame == BASE:
            return pts                       # already there; nothing to do
        if pts[0].frame != CAMERA:
            raise CellError(f"hand-eye transforms camera points, "
                            f"not {pts[0].frame!r} ones")
        flange = batch.flange if isinstance(batch, Batch) else None
        return move_points(self.base_from_camera(flange), pts, BASE)

    def to_camera(self, point: Point, flange: Pose | None) -> Point:
        """The other direction -- used to draw a known base point back on the
        image, which is the cheapest calibration sanity check there is."""
        T = invert(self.base_from_camera(flange))
        return move_points(T, [point], CAMERA)[0]

    # ------------------------------------------------------------- reporting
    @property
    def offset_mm(self) -> tuple[float, float, float]:
        """Where the camera sits relative to the flange, in mm."""
        t = self.matrix[:3, 3]
        return (float(t[0]), float(t[1]), float(t[2]))

    def describe(self) -> str:
        x, y, z = self.offset_mm
        return (f"hand-eye: {self.mode}, camera at "
                f"[{x:+.1f}, {y:+.1f}, {z:+.1f}] mm in the flange frame"
                + (f", rms {self.rms_mm:.1f} mm over {self.n_poses} poses"
                   if self.n_poses else "")
                + f"  ({self.path.name})")

    def warnings(self) -> list[str]:
        """Things worth saying out loud before a sniffer trusts these numbers."""
        out = []
        if self.rms_mm > 3.0:
            out.append(
                f"the calibration reports {self.rms_mm:.1f} mm rms -- that is "
                f"coarse for a sniffer. If the board had an odd number of inner "
                f"corners on BOTH sides, half the captures may have the origin "
                f"flipped; a 9x6 board or ChArUco fixes it.")
        if self.board:
            cols, rows = self.board.get("cols"), self.board.get("rows")
            if cols and rows and cols % 2 == 1 and rows % 2 == 1:
                out.append(
                    f"the calibration board is {cols}x{rows} inner corners -- "
                    f"both odd, so OpenCV can pick either end as the origin and "
                    f"the captures split into two clusters. Re-calibrate with "
                    f"one even side before trusting millimetres.")
        if self.n_poses and self.n_poses < 12:
            out.append(f"only {self.n_poses} calibration poses -- 15 to 20 "
                       f"spread over the work area is the usual minimum")
        return out
