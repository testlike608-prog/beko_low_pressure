"""
The leak tester, behind the same two functions.

    import sniffer

    sniffer.start()
    result = sniffer.test(label="weld3")     -> ("pass" | "fail" | "error", value)
    sniffer.stop()

Today this is a placeholder with two working implementations -- Fake (for
building everything else) and Digital (trigger an output, wait for an input,
which is how the Inficon is actually wired). When the real Galileo / Inficon
protocol goes in, it is one more file here and one line in REGISTRY. The
robot, the camera and the model do not change.

The important shape: test() NEVER raises for a leak. A leaking weld is a
result, not an error. It raises only when the TESTER itself cannot be talked
to, because those two need different reactions from the operator.
"""

from __future__ import annotations

import time

from core import CellError

PASS, FAIL, ERROR, UNTESTED = "pass", "fail", "error", "not tested"


class Sniffer:
    name = "sniffer"

    def open(self) -> None: ...

    def close(self) -> None: ...

    def test(self, label: str = "", timeout_s: float = 10.0):
        """-> (verdict, value). Verdict is one of PASS / FAIL / ERROR."""
        raise NotImplementedError

    def describe(self) -> str:
        return self.name


class Fake(Sniffer):
    """
    Always passes, unless `fail_labels` names the point.

        Fake()
        Fake(fail_labels=["weld3"], seconds=0.2)
    """

    name = "fake sniffer"

    def __init__(self, seconds=0.2, fail_labels=None, value=1.2e-6):
        self.seconds = float(seconds)
        self.fail_labels = set(fail_labels or ())
        self.value = float(value)

    def test(self, label: str = "", timeout_s: float = 10.0):
        time.sleep(self.seconds)
        if label in self.fail_labels:
            return FAIL, self.value * 50
        return PASS, self.value


class Digital(Sniffer):
    """
    The wiring this cell actually has: one output starts the test, one input
    says the tester is ready, one input carries the verdict.

    Reads and writes go through the ROBOT controller's I/O, because that is
    where the module is wired. Swap in a PLC or a Modbus module by writing
    another class here -- the cycle above does not notice.
    """

    name = "digital sniffer"

    def __init__(self, trigger_out=0, ready_in=0, pass_in=1, fail_in=2,
                 settle_s=0.3, poll_s=0.05):
        self.trigger_out = int(trigger_out)
        self.ready_in = int(ready_in)
        self.pass_in = int(pass_in)
        self.fail_in = int(fail_in)
        self.settle_s = float(settle_s)
        self.poll_s = float(poll_s)

    def _io(self):
        import robot
        arm = robot.current()
        if not arm.has_io:
            raise CellError(f"{arm.name} has no digital I/O, so the digital "
                            f"sniffer cannot be wired through it")
        return arm

    def open(self) -> None:
        arm = self._io()
        arm.write_output(self.trigger_out, 0)          # known state, not a pulse
        print(f"  sniffer: digital, trigger DO{self.trigger_out}, "
              f"ready DI{self.ready_in}, pass DI{self.pass_in}, "
              f"fail DI{self.fail_in}")

    def close(self) -> None:
        try:
            self._io().write_output(self.trigger_out, 0)
        except Exception:
            pass

    def test(self, label: str = "", timeout_s: float = 10.0):
        arm = self._io()

        deadline = time.time() + timeout_s
        while not arm.read_input(self.ready_in):
            if time.time() > deadline:
                return ERROR, None
            time.sleep(self.poll_s)

        arm.write_output(self.trigger_out, 1)
        time.sleep(self.settle_s)
        arm.write_output(self.trigger_out, 0)

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if arm.read_input(self.pass_in):
                return PASS, None
            if arm.read_input(self.fail_in):
                return FAIL, None
            time.sleep(self.poll_s)
        return ERROR, None


REGISTRY = {
    "fake": ("sniffer", "Fake"),
    "digital": ("sniffer", "Digital"),
}


def kinds() -> list[str]:
    return sorted(REGISTRY)


def build(config: dict | None = None) -> Sniffer:
    if config is None:
        import settings
        config = getattr(settings, "SNIFFER", {"kind": "fake"})
    config = dict(config)
    kind = config.pop("kind", "fake")
    if kind not in REGISTRY:
        raise CellError(f"unknown sniffer kind {kind!r}. "
                        f"Known kinds: {', '.join(kinds())}")
    module_name, class_name = REGISTRY[kind]
    import importlib
    return getattr(importlib.import_module(module_name), class_name)(**config)


_sniffer: Sniffer | None = None


def start(config: dict | None = None) -> Sniffer:
    global _sniffer
    if _sniffer is not None:
        return _sniffer
    s = build(config)
    s.open()
    _sniffer = s
    return s


def stop() -> None:
    global _sniffer
    if _sniffer is None:
        return
    try:
        _sniffer.close()
    except Exception as e:
        print(f"  !! closing the sniffer raised: {e}")
    finally:
        _sniffer = None


def current() -> Sniffer:
    if _sniffer is None:
        raise CellError("the sniffer has not been started -- call sniffer.start()")
    return _sniffer


def is_running() -> bool:
    return _sniffer is not None


def test(label: str = "", timeout_s: float = 10.0):
    return current().test(label, timeout_s)


def describe(config: dict | None = None) -> str:
    if _sniffer is not None:
        return _sniffer.describe()
    try:
        return build(config).describe() + " (not open)"
    except CellError as e:
        return f"sniffer misconfigured: {e}"
