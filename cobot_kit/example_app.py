"""
Your app.py's skeleton, filled in with cobot_kit -- as an EXAMPLE to copy from.
app.py itself is not touched. Every comment below is the one from app.py.

    python -m cobot_kit.example_app            simulator + fake camera + fixed points
    python -m cobot_kit.example_app --real     the real FR5 + RealSense + weld.pt

db / scanner / telegram are left as the comments they are in app.py -- those
are your modules; this file only shows where the cobot side plugs in.
"""

from __future__ import annotations

import sys
import threading
import time

from cobot_kit import (AiModel, Camera, CellError, FixedPoints, Locator,
                       Robot, cycle)


class App:
    def __init__(self, scanner_ip=None, real=False, trigger_input=0):
        self.scannr_ip = scanner_ip
        self.robot_is_connected = False
        self.trigger_input = trigger_input          # DI on the cobot controller

        if real:
            self.robot = Robot()                    # settings.py values
            self.camera = Camera()
            self.ai = AiModel()                     # settings.AI["weights"]
        else:
            self.robot = Robot(kind="simulator", speed_factor=0.0)
            self.camera = Camera(backend="fake")
            self.ai = FixedPoints([(640, 360), (740, 380), (540, 380)])
        self.locator = Locator()
        self._loop = None
        self._running = threading.Event()

    def _start(self):
        #connect to camera
        self.camera.start()
        #connect to db
        #connect to cobot
        self.robot.start()
        self.ai.start()
        self.robot_is_connected = True
        #connect to scanner client
        #start lestening for scanner
        #start the main loop function (_running_loop)
        self._running.set()
        self._loop = threading.Thread(target=self._running_loop, daemon=True)
        self._loop.start()

    def _stop(self):
        #stop the main loop function (_running_loop)
        self._running.clear()
        self.robot_is_connected = False
        self.robot.halt()                           # stops a move in progress
        if self._loop is not None:
            self._loop.join(timeout=5)
        #stop camera connection
        self.ai.stop()
        self.camera.stop()
        #stop the scanner connection
        self.robot.stop()                           # home, MANUAL, disconnect

    def _running_loop(self):
        last = 0
        while self.robot_is_connected and self._running.is_set():
            #check te inpout trigger
            now = self.robot.read_input(self.trigger_input)
            #if the input equals to one start the sequance
            if now == 1 and last == 0:              # rising edge = once per part
                try:
                    self._start_sequance()
                except CellError as e:
                    print(f"\n  !! {e}\n")
            last = now
            time.sleep(0.05)

    def _start_sequance(self):
        #move the robot to cap positions
        #trig the camera to cap images and save it in capture folder
        shots = cycle.capture(self.robot, self.camera)
        #send the cap image to the ai model and give me x,y pixel fro image
        shots = cycle.detect(shots, self.ai)
        #send the selected welding points from ai to camera to detect the x,y,z for the copot
        points = cycle.locate(shots, self.locator)
        #start the cycle function
        return self._robot_cycle(points)

    def _robot_cycle(self,points_array):
        #start the loop for each point
        # لكل نقطة: approach -> النقطة -> يطلّع output -> يستنى input -> يرجع ورا
        import time
        from cobot_kit import CellError, cycle

        OUTPUT_NO   = 0       # الـ DO اللي بيطلع لما الروبوت يوصل النقطة
        INPUT_NO    = 0       # الـ DI اللي بنستناه قبل ما نروح للنقطة اللي بعدها
        WAIT_S      = 30.0    # أقصى وقت نستنى فيه الـ input (None = للأبد)
        APPROACH_MM = 60.0    # يقف قبل النقطة بالمسافة دي على محور الـ tool
        CLEARANCE_MM = 0.0    # يقف قبل النقطة نفسها بالمسافة دي (الـ sniffer مش بيلمس)
        SPEED       = 20.0    # سرعة الروح للـ approach، %
        SLOW        = 8.0     # سرعة آخر حتة ناحية النقطة والرجوع، %
        DWELL_S     = 0.5     # يستنى الذراع تهدى قبل الـ output

        robot = self.robot
        orientation = robot.pose().rpy     # الـ tool يفضل بنفس الاتجاه اللي هو عليه دلوقتي
        results = []

        for i, point in enumerate(points_array, 1):
            if not self.robot_is_connected:        # _stop اتنده -- نقف
                break

            pose     = cycle.point_pose(point, orientation, CLEARANCE_MM)
            approach = cycle.approach_pose(pose, APPROACH_MM)
            print(f"  -> point {i}: {pose}")

            try:
                # 1. روح للنقطة
                robot.move_to(approach, vel=SPEED, label=f"point {i} approach")
                robot.move_to(pose, vel=SLOW, linear=True, label=f"point {i}")
                time.sleep(DWELL_S)

                # 2. طلّع الـ output
                robot.write_output(OUTPUT_NO, 1)

                # 3. استنى الـ input
                got_it = robot.wait_input(INPUT_NO, 1, timeout_s=WAIT_S,
                                          cancel=lambda: not self.robot_is_connected)
                robot.write_output(OUTPUT_NO, 0)
                if not got_it:
                    print(f"  !! point {i}: DI{INPUT_NO} did not come in {WAIT_S} s")

                # 4. ارجع ورا على نفس الخط، وبعدها النقطة اللي بعدها
                robot.move_to(approach, vel=SLOW, linear=True, label=f"point {i} retreat")
                results.append({"point": i, "xyz": point.xyz,
                                "ok": got_it, "note": "" if got_it else "input timeout"})

            except CellError as e:
                # نقطة مش reachable أو الكونترولر رفض: سجّلها وكمّل على اللي بعدها
                print(f"  !! point {i} skipped: {e}")
                try:
                    robot.write_output(OUTPUT_NO, 0)
                except CellError:
                    pass
                results.append({"point": i, "xyz": point.xyz, "ok": False, "note": str(e)})

        return results

if __name__ == "__main__":
    real = "--real" in sys.argv
    app = App(real=real)
    try:
        app._start()
        if not real:
            # no trigger wired in the simulator: pull the input high once
            app.robot.arm.io[app.trigger_input] = 1
        time.sleep(3 if not real else 3600)
    except KeyboardInterrupt:
        pass
    finally:
        app._stop()
