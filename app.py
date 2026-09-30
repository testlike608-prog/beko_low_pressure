"""
app.py -- Beko low-pressure cobot leak-test cell, following beko_cobot_(3).drawio.

Login / roles / "Start Button Pressed?" are the FRONT END's job. Everything
after the start button is here:

    _start()            Initialization variables and network connections
                        (camera / cobot / DB ... each failure -> telegram)
    _running_loop()     Read I/O == 1 ?  (the cobot controller's DI) -> _start_sequance()
    _start_sequance()   start timer -> Trig Scanners -> dummy -> SKU -> recipe
                        -> capture + AI (retry <= 2) -> ready-to-start check
                        -> _robot_cycle -> final results -> homing
                        -> retest same device (n < M) / calibration (c == C)
    _robot_cycle()      for each point: move -> Robot_Arrived -> trigger tester
                        -> Wait_Tester_Reply -> Save_Result
    _stop()             everything down, never raises

Flowchart letters (legend):
    n  how many cycles were repeated for the same device
    M  maximum cycles for the same device            -> self.M_MAX_SAME
    N  number of expected welding points (recipe)
    c  number of performed cycles
    C  cycles required before a calibration          -> self.C_CALIBRATION

Run without the front end:   python app.py
"""

import csv
import json
import os
import threading
import time

import httpx

import telegram_ask as tel
import db
import client as cl
from cobot_kit import AiModel, Camera, CellError, Locator, Robot, cycle
from cobot_kit import settings as kit_settings

HERE = os.path.dirname(os.path.abspath(__file__))
SETTINGS_FILE = os.path.join(HERE, "settings.json")

#db.auto_connect_db()
#db.get_product_number(dummy_number=test)
#tel.ask()


class App():
    def __init__(self, scanner_ip, scanner_port, robot=None, camera=None, ai=None):
        self.scannr_ip= scanner_ip
        self.scanner_port= scanner_port
        self.robot_is_connected = False

        # ---- hardware (pass your own for testing: Robot(kind="simulator"), ...)
        self.robot   = robot  or Robot()
        self.camera  = camera or Camera()
        self.ai      = ai     or AiModel()          # weights from cobot_kit/settings.py
        self.locator = None                          # loaded in _start (handeye.json)
        self.SCAN_TIMEOUT_S = 5.0    # السكانر لازم يرد في الوقت ده وإلا "failed to scan"
        self.scanner = cl.TCPClient(scanner_ip, scanner_port, timeout=self.SCAN_TIMEOUT_S)
        self.scanner.name = "scanner"

        # ---- robot cycle
        self.WAIT_S      = 30.0    # أقصى وقت نستنى فيه رد الـ tester (None = للأبد)
        self.APPROACH_MM = 60.0    # يقف قبل النقطة بالمسافة دي على محور الـ tool
        self.CLEARANCE_MM = 0.0    # يقف قبل النقطة نفسها بالمسافة دي (الـ sniffer مش بيلمس)
        self.SPEED       = 20.0    # سرعة الروح للـ approach، %
        self.SLOW        = 8.0     # سرعة آخر حتة ناحية النقطة والرجوع، %
        self.DWELL_S     = 0.5     # يستنى الذراع تهدى قبل الـ output
        self.PULSE_S     = 0.3     # طول نبضة Send_Trigger(Leak_Tester)

        # ---- I/O: كله على الـ DI / DO بتاع كونترولر الروبوت
        # !! الأرقام دي مثال -- غيّرها حسب الـ wiring الحقيقي
        self.DI_START          = 0   # Read I/O == 1 (قطعة جاهزة)
        self.DI_TESTER_READY   = 1   # Galileo/INFICON: ready to start
        self.DI_TESTER_DONE    = 2   # Wait_Tester_Reply()
        self.DI_TESTER_PASS    = 3   # نتيجة النقطة: 1 = مفيش leak
        self.DI_CALIB_DONE     = 4   # Read I/O == 1 بعد ما الكاليبريشن تخلص
        self.DO_ROBOT_ARRIVED  = 0   # Send_Signal(Robot_Arrived)
        self.DO_TESTER_TRIGGER = 1   # Send_Trigger(Leak_Tester)

        # ---- scanner
        self.SCANNER_TRIGGER = "start"   # الأمر اللي بيخلي السكانر يقرا -- زي ما متظبط في السكانر
        self.SCANNER_NO_READ = ("", "noread", "no read", "error")

        # ---- flowchart limits
        self.CAPTURE_RETRIES = 2         # "if counter <= 2" -> صوّر تاني
        self.M_MAX_SAME      = 3         # M: أقصى عدد مرات لنفس الجهاز
        self.C_CALIBRATION   = 500       # C: كل كام cycle يعمل كاليبريشن
        self.CALIBRATION_TYPE   = "manual"
        self.CALIBRATION_JOINTS = [0.0, -20.0, -90.0, -70.0, 90.0, 0.0]   # !! غيّرها
        self.RECIPES_DIR     = "recipes"   # relative = جنب app.py
        self.UPLOAD_RESULTS  = False     # True = يرفع النتيجة على DB2 (db.upload_tests_result_to_db)
        self.TEST_NAME       = "LowPressureLeakTest"

        # ---- state
        self._running = threading.Event()
        self._loop = None
        self.on_event = None             # callback(event, data) -- للـ Socket.IO بعدين
        self._reset_variables()

        pass

    def _reset_variables(self):
        """Initialization variables."""
        self.n = 0                  # repeats for the same device
        self.c = 0                  # performed cycles
        self.N = 0                  # expected welding points (from the recipe)
        self.last_dummy = None
        self._t0 = None             # timer
        self.last_results = []

    # ============================================================ settings
    #: Besides every UPPERCASE variable in __init__, the GUI may also change these.
    EXTRA_SETTINGS = ("scannr_ip", "scanner_port")
    #: UPPERCASE but NOT a setting: N comes from the recipe on every part.
    NOT_SETTINGS = ("N",)

    def _settings_keys(self):
        """Every setting in settings.json = every UPPERCASE variable in __init__
        (+ EXTRA_SETTINGS). A new constant in __init__ is picked up automatically."""
        return sorted(k for k in vars(self)
                      if k.isupper() and k not in self.NOT_SETTINGS) + list(self.EXTRA_SETTINGS)

    def _get_all_settings(self):
        """Current values as a dict -- what the GUI shows."""
        out = {}
        for k in self._settings_keys():
            v = getattr(self, k)
            out[k] = list(v) if isinstance(v, tuple) else v
        return out

    def _save_all_settings(self, values=None, path=SETTINGS_FILE):
        """Write settings.json (all of it). values=None -> the current values."""
        data = self._get_all_settings() if values is None else values
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        os.replace(tmp, path)          # never leaves a half-written file behind

    def _set_all_settings(self, path=SETTINGS_FILE):
        """
        settings.json -> every variable in __init__. Called by _start(), so a
        change from the GUI takes effect on the next Start.

        * missing or empty file  -> written with the current defaults
        * a key the file lacks   -> keeps its default, and is added to the file
        * an unknown key         -> ignored, with a warning (typo, or an old key)
        * a wrong type / value   -> CellError naming every bad key; NOTHING is
                                    applied, so the cell never runs half-updated
        """
        try:
            with open(path, encoding="utf-8-sig") as f:
                text = f.read().strip()
        except FileNotFoundError:
            text = ""
        if not text:
            self._save_all_settings(path=path)
            print(f"  settings: {os.path.basename(path)} was empty -- wrote the defaults")
            return

        try:
            data = json.loads(text)
        except json.JSONDecodeError as e:
            raise CellError(f"{os.path.basename(path)} is not valid JSON: {e}")
        if not isinstance(data, dict):
            raise CellError(f"{os.path.basename(path)} must be one JSON object {{...}}")

        keys = set(self._settings_keys())
        new, errors = {}, []
        for key, value in data.items():
            if key not in keys:
                print(f"  !! settings: unknown key '{key}' ignored")
                continue
            try:
                new[key] = self._coerce_setting(key, value, getattr(self, key))
            except (TypeError, ValueError) as e:
                errors.append(f"{key}: {e}")
        if errors:
            raise CellError("bad values in settings.json -- nothing applied:\n    "
                            + "\n    ".join(errors))

        for key, value in new.items():
            setattr(self, key, value)

        # the scanner client was built in __init__ -- give it the new address
        self.scanner.ip = self.scannr_ip
        self.scanner.port = int(self.scanner_port)
        self.scanner.timeout = self.SCAN_TIMEOUT_S

        missing = keys - set(data)
        if missing:
            self._save_all_settings(path=path)
            print(f"  settings: added {', '.join(sorted(missing))} to the file")
        print(f"  settings: {len(new)} value(s) loaded from {os.path.basename(path)}")

    @staticmethod
    def _coerce_setting(key, value, default):
        """The JSON value in the type of the default. Raises with a sentence."""
        if key == "WAIT_S" and value is None:              # None = wait forever
            return None
        if isinstance(default, bool):                      # before int: bool IS an int
            if not isinstance(value, bool):
                raise TypeError(f"must be true / false, got {value!r}")
            return value
        if isinstance(default, int) and key != "WAIT_S":
            if isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or float(value) != int(value):
                raise TypeError(f"must be a whole number, got {value!r}")
            if key.startswith(("DI_", "DO_")) and not 0 <= int(value) <= 15:
                raise ValueError(f"I/O number must be 0..15, got {value}")
            return int(value)
        if isinstance(default, (int, float)) or key == "WAIT_S":
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TypeError(f"must be a number, got {value!r}")
            if value < 0:
                raise ValueError(f"must not be negative, got {value}")
            return float(value)
        if isinstance(default, str):
            if not isinstance(value, (str, int)):
                raise TypeError(f"must be text, got {value!r}")
            return str(value)
        if isinstance(default, (list, tuple)):
            if not isinstance(value, list):
                raise TypeError(f"must be a list [...], got {value!r}")
            if key == "CALIBRATION_JOINTS":
                if len(value) != 6 or not all(isinstance(v, (int, float))
                                              and not isinstance(v, bool) for v in value):
                    raise ValueError(f"must be 6 numbers (degrees), got {value!r}")
                return [float(v) for v in value]
            return tuple(value) if isinstance(default, tuple) else list(value)
        return value


    # ================================================================ start
    def _start(self):
        """
        Initialization variables and network connections. Every check that
        fails sends a telegram alarm. Returns True when the loop is running.
        """
        if self._running.is_set():
            return True

        # settings.json -> the variables in __init__ (the GUI edits the file)
        try:
            self._set_all_settings()
        except CellError as e:
            self._notify(f"settings: {e}")
            return False

        self._reset_variables()
        ok = True

        #connect to camera -- "is camera connected?"
        try:
            self.camera.start()
        except CellError as e:
            self._notify(f"failed to connect camera: {e}")
            ok = False

        #connect to cobot -- "is cobot connected?"
        try:
            self.robot.start()
            self.robot_is_connected = True
        except CellError as e:
            self._notify(f"failed to connect cobot: {e}")
            ok = False

        #connect to db -- "Failed to connect DB"
        db.auto_connect_db()
        if not db.connected1:
            self._notify("failed to connect DB (DB1 -- dummy / SKU lookup)")
            ok = False
        elif self.UPLOAD_RESULTS and not db.connected2:
            self._notify("failed to connect DB2 -- results will not be uploaded")

        # the model + hand-eye (not boxes in the flowchart, but nothing works without them)
        try:
            self.ai.start()
            self.locator = Locator()
        except CellError as e:
            self._notify(f"failed to load the AI model / hand-eye: {e}")
            ok = False

        #connect to scanner client
        if not self.scanner.connect():
            self._notify(f"failed to connect scanner {self.scannr_ip}:{self.scanner_port}")
            ok = False

        if not ok:
            self._stop()
            return False

        self.scanner.start_reconnection_watchdog()

        #start the main loop function (_running_loop)
        self._running.set()
        self._loop = threading.Thread(target=self._running_loop, name="cell-loop",
                                      daemon=True)
        self._loop.start()
        self._emit("state", {"running": True})
        return True

    # ================================================================= stop
    def _stop(self):
        """Everything down. Safe twice, safe after a failed _start. Never raises."""
        #stop the main loop function (_running_loop)
        self._running.clear()
        self.robot_is_connected = False
        try:
            self.robot.halt()                       # stops a move in progress
        except Exception:
            pass
        if self._loop is not None and self._loop is not threading.current_thread():
            self._loop.join(timeout=10)
        self._loop = None

        for name, fn in (("tester trigger off", lambda: self._write(self.DO_TESTER_TRIGGER, 0)),
                         ("robot arrived off",  lambda: self._write(self.DO_ROBOT_ARRIVED, 0)),
                         ("ai",                 self.ai.stop),
                         #stop camera connection
                         ("camera",             self.camera.stop),
                         #stop the scanner connection
                         ("scanner",            self.scanner.stop),
                         ("robot",              self.robot.stop)):      # home + MANUAL
            try:
                fn()
            except Exception as e:
                print(f"  !! stop {name}: {e}")
        self._emit("state", {"running": False})


    # =========================================================== main loop
    def _running_loop(self):
        last = 0
        while self.robot_is_connected and self._running.is_set():
            try:
                #check te inpout trigger -- Read I/O == 1?
                now = self._read(self.DI_START)
                #if the input equals to one start the sequance  (one part = one rising edge)
                if now == 1 and last == 0:
                    self._start_sequance()
                last = now
            except CellError as e:
                self._notify(f"cycle error: {e}")
                last = 1                              # wait for the signal to drop again
                time.sleep(1.0)
            except Exception as e:                    # never let the loop die silently
                self._notify(f"unexpected error in the loop: {e!r}")
                time.sleep(1.0)
            time.sleep(0.05)


    # ============================================================ sequence
    def _start_sequance(self):
        while self._running.is_set():
            self._t0 = time.time()                                   # start timer
            self._emit("cycle_start", {"c": self.c, "n": self.n})

            # ---- Trig Scanners -> dummy scanned?
            dummy = self._scan_dummy()
            if dummy is None:
                self._notify("failed to scan")
                return
            # is the new dummy = the last dummy?
            self.n = self.n + 1 if dummy == self.last_dummy else 0
            self.last_dummy = dummy
            self._emit("dummy", {"dummy": dummy, "n": self.n})

            # ---- dummy found in db? -> Got SKU From DB
            sku, msg, _level = db.get_product_number(dummy)
            if not sku:
                self._notify(f"dummy not found in db: {dummy} ({msg})")
                return
            sku = str(sku).strip()

            # ---- csv found for sku? -> Load Recipe
            recipe = self._load_recipe(sku)
            if recipe is None:
                self._notify(f"csv not found for sku {sku} (dummy {dummy})")
                return
            self.N = recipe["welding_points"]
            self._emit("recipe", {"sku": sku, "N": self.N})

            # ---- capture + AI, retried  (Length(Points_Array) > 0 and == N)
            points = self._find_points(recipe)
            if points is None:
                self._notify(f"fail to detect all welding points -- dummy {dummy}, "
                             f"sku {sku}, expected {self.N}")
                self._go_home()
                return

            # ---- check if ready to start is presence  (Galileo -> INFICON branch)
            if self._read(self.DI_TESTER_READY) != 1:
                self._notify("ready to start signal not found (Galileo / INFICON)")
                return

            #start the cycle function
            results = self._robot_cycle(points)

            # ---- Print_Final_Results / c+1 / Reset timer
            self.c += 1
            seconds = time.time() - self._t0
            self._t0 = None                                          # Reset timer
            passed = self._print_final_results(dummy, sku, results, seconds)

            # ---- is the result = 1?  /  is the n = M?
            if passed or self.n >= self.M_MAX_SAME:
                self._go_home()                                      # Move Cobot to homing
                if self.c >= self.C_CALIBRATION:                     # Is c = C?
                    self._calibration()
                return                                               # -> Read I/O
            print(f"  result failed, n={self.n} < M={self.M_MAX_SAME} "
                  f"-- testing the same device again")
            # no -> back to "start timer" for the same device


    def _find_points(self, recipe):
        """
        Move Cobot for capture position -> Trig Camera -> Points_Array =
        Vision_Processing(Image) -> Length > 0 and == N ?  Retried while
        counter <= CAPTURE_RETRIES. Returns the base-frame points or None.
        """
        counter = 0
        while self._running.is_set():
            #move the robot to cap positions + trig the camera + save in capture folder
            shots = cycle.capture(self.robot, self.camera, recipe["capture_positions"])
            #send the cap image to the ai model and give me x,y pixel
            shots = cycle.detect(shots, self.ai)
            #send the selected welding points to camera to detect the x,y,z for the cobot
            points = cycle.locate(shots, self.locator)
            self._emit("points", {"found": len(points), "expected": self.N,
                                  "images": [str(s.image_path) for s in shots
                                             if s.image_path]})

            if len(points) > 0 and len(points) == self.N:
                return points
            self._print_error(f"found {len(points)} welding points, the recipe "
                              f"expects {self.N}")
            counter += 1                                              # counter = i+1
            if counter > self.CAPTURE_RETRIES:                        # if counter <= 2
                return None
        return None


    def _robot_cycle(self,points_array):

        #start the loop for each point
        # لكل نقطة: approach -> النقطة -> Robot_Arrived -> trigger الـ tester -> يستنى الرد -> يرجع ورا

        orientation = self.robot.pose().rpy     # الـ tool يفضل بنفس الاتجاه اللي هو عليه دلوقتي
        results = []

        for i, point in enumerate(points_array, 1):
            if not self.robot_is_connected:        # _stop اتنده -- نقف
                break

            pose     = cycle.point_pose(point, orientation, self.CLEARANCE_MM)
            approach = cycle.approach_pose(pose, self.APPROACH_MM)
            print(f"  -> point {i}: {pose}")

            try:
                # 1. Move_Cobot(Points_Array[i])
                self.robot.move_to(approach, vel=self.SPEED, label=f"point {i} approach")
                self.robot.move_to(pose, vel=self.SLOW, linear=True, label=f"point {i}")
                time.sleep(self.DWELL_S)

                # 2. Send_Signal(Robot_Arrived)
                self._write(self.DO_ROBOT_ARRIVED, 1)

                # 3. Send_Trigger(Leak_Tester)
                self._pulse(self.DO_TESTER_TRIGGER, self.PULSE_S)

                # 4. Wait_Tester_Reply() == True ?   NO -> wait
                got_it = self._wait(self.DI_TESTER_DONE, 1, timeout_s=self.WAIT_S)
                leak = "pass" if got_it and self._read(self.DI_TESTER_PASS) == 1 else \
                       "fail" if got_it else "no reply"
                self._write(self.DO_ROBOT_ARRIVED, 0)
                if not got_it:
                    print(f"  !! point {i}: the tester did not reply in {self.WAIT_S} s")

                # 5. ارجع ورا على نفس الخط، وبعدها النقطة اللي بعدها
                self.robot.move_to(approach, vel=self.SLOW, linear=True, label=f"point {i} retreat")

                # 6. Save_Result(Points_Array[i])
                results.append(self._save_result(i, point, got_it and leak == "pass",
                                                 leak, "" if got_it else "tester timeout"))

            except CellError as e:
                # نقطة مش reachable أو الكونترولر رفض: سجّلها وكمّل على اللي بعدها
                print(f"  !! point {i} skipped: {e}")
                for sig in (self.DO_TESTER_TRIGGER, self.DO_ROBOT_ARRIVED):
                    try:
                        self._write(sig, 0)
                    except CellError:
                        pass
                results.append(self._save_result(i, point, False, "not tested", str(e)))

        return results


    # ============================================================ results
    def _save_result(self, i, point, ok, leak, note):
        r = {"point": i, "label": point.label, "xyz": [round(v, 1) for v in point.xyz],
             "ok": bool(ok), "leak": leak, "note": note}
        self._emit("point_result", r)
        return r

    def _print_final_results(self, dummy, sku, results, seconds):
        """Print_Final_Results(). Returns True when every expected point passed."""
        passed = len(results) == self.N and all(r["ok"] for r in results)
        failed = [r for r in results if not r["ok"]]
        print("\n  " + "-" * 60)
        print(f"  dummy {dummy}  sku {sku}  cycle {self.c}  repeat n={self.n}  "
              f"{seconds:.1f} s")
        for r in results:
            print(f"   {r['point']:>2}. {r['label'] or 'point':<12} {r['leak']:<10} "
                  f"{r['xyz']}  {r['note']}")
        print(f"  RESULT: {'PASS' if passed else 'FAIL'}   "
              f"({len(results) - len(failed)}/{self.N} ok)")
        print("  " + "-" * 60 + "\n")
        self.last_results = results
        self._emit("final_result", {"dummy": dummy, "sku": sku, "passed": passed,
                                    "seconds": round(seconds, 1), "c": self.c,
                                    "n": self.n, "results": results})
        if self.UPLOAD_RESULTS:
            failed_names = ",".join(r["label"] or f"point{r['point']}" for r in failed)
            db.upload_tests_result_to_db(dummy, self.TEST_NAME,
                                         "PASS" if passed else "FAIL",
                                         failed_names, self.scanner)   # any client with _log_add
        return passed

    def _print_error(self, text):
        print(f"  !! {text}")
        self._emit("error", {"text": text})


    # ========================================================= calibration
    def _calibration(self):
        """Is c = C? -> calibration mode ON."""
        self._emit("calibration", {"c": self.c, "C": self.C_CALIBRATION})
        print(f"\n  calibration mode ON (c={self.c} = C={self.C_CALIBRATION})")
        if self.CALIBRATION_TYPE != "manual":
            self._notify(f"calibration is due, type '{self.CALIBRATION_TYPE}' "
                         f"is not implemented -- counter reset")
            self.c = 0
            return
        # Move Cobot to calibration position -> manual mode -> telegram
        self.robot.move_joints(self.CALIBRATION_JOINTS, label="calibration position")
        self.robot.set_manual()
        self._notify(f"calibration needed: the cobot is at the calibration position "
                     f"in MANUAL mode after {self.c} cycles. Set DI{self.DI_CALIB_DONE} "
                     f"when finished.")
        # Read I/O == 1? / Is the calibration process finished?  (wait)
        self._wait(self.DI_CALIB_DONE, 1, timeout_s=None)
        if not self._running.is_set():
            return
        self.robot.set_automatic()
        self.c = 0
        self._emit("calibration_done", {})


    # ============================================================ helpers
    def _go_home(self):
        try:
            self.robot.go_home()
        except CellError as e:
            self._notify(f"could not move the cobot home: {e}")

    def _scan_dummy(self):
        """Trig Scanners -> the dummy text, or None (no read / no answer)."""
        resp = self.scanner.send_request(self.SCANNER_TRIGGER)
        if not resp:
            return None
        text = resp.decode("utf-8", errors="ignore").strip()
        if text.lower() in self.SCANNER_NO_READ:
            return None
        return text

    def _load_recipe(self, sku):
        """
        recipes/<SKU>.csv -- one "key, value..." per line, # = comment:

            welding_points, 6
            capture_joints, 0, -20, -90, -70, 90, 0     (one line per position)
            capture_pose, 734, -426, -100, 180, 0, 175

        No capture line -> CAPTURE_POSITIONS from cobot_kit/settings.py.
        Returns None when the file does not exist.
        """
        path = os.path.join(HERE, self.RECIPES_DIR, f"{sku}.csv")   # an absolute RECIPES_DIR wins
        if not os.path.exists(path):
            return None
        recipe = {"sku": sku, "welding_points": None, "capture_positions": []}
        with open(path, newline="", encoding="utf-8-sig") as f:
            for n, row in enumerate(csv.reader(f), 1):
                row = [c.strip() for c in row]
                if not row or not row[0] or row[0].startswith("#"):
                    continue
                key, values = row[0].lower(), [v for v in row[1:] if v]
                try:
                    if key == "welding_points":
                        recipe["welding_points"] = int(values[0])
                    elif key == "capture_joints":
                        recipe["capture_positions"].append(
                            {"joints": [float(v) for v in values[:6]]})
                    elif key == "capture_pose":
                        recipe["capture_positions"].append(
                            {"pose": [float(v) for v in values[:6]]})
                except (ValueError, IndexError):
                    raise CellError(f"{path} line {n} is not valid: {row}")
        if not recipe["welding_points"]:
            raise CellError(f"{path} has no 'welding_points, N' line")
        if not recipe["capture_positions"]:
            recipe["capture_positions"] = list(kit_settings.CAPTURE_POSITIONS)
        return recipe

    # ---- I/O: the cobot controller's DI / DO ----------------------------
    def _read(self, di):
        return self.robot.read_input(di)

    def _write(self, do, value):
        self.robot.write_output(do, value)

    def _pulse(self, do, seconds):
        self._write(do, 1)
        time.sleep(seconds)
        self._write(do, 0)

    def _wait(self, di, value, timeout_s=None):
        """Wait for DI == value. False on timeout or when _stop was called."""
        return self.robot.wait_input(di, value, timeout_s=timeout_s, poll_s=0.05,
                                     cancel=lambda: not self._running.is_set())

    # ---- telegram / events ---------------------------------------------
    def _notify(self, text):
        """Send a telegram notification alarm to the operator. Never blocks, never raises."""
        print(f"  [ALARM] {text}")
        self._emit("alarm", {"text": text})
        if not tel.BOT_TOKEN:
            print("  (telegram: no BOT_TOKEN in .env -- not sent)")
            return

        def send():
            try:
                httpx.post(f"{tel.API}/sendMessage", timeout=10,
                           json={"chat_id": tel.GROUP_CHAT_ID, "text": text})
            except Exception as e:
                print(f"  !! telegram: {e}")
        threading.Thread(target=send, daemon=True).start()

    def _emit(self, event, data):
        """Hook for the front end (Socket.IO later): set app.on_event = fn(event, data)."""
        if self.on_event is not None:
            try:
                self.on_event(event, data)
            except Exception as e:
                print(f"  !! on_event({event}): {e}")


##################################################################

if __name__ == "__main__":
    # !! IPs are examples -- put the real ones
    app = App(scanner_ip="192.168.1.20", scanner_port=2001)
    if app._start():
        print("\n  running -- Ctrl+C to stop\n")
        try:
            while app._running.is_set():
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
    app._stop()
