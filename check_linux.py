"""
check_linux.py -- is this Jetson (JetPack 6.2) / Linux PC ready to run beko_low_pressure?

    python3 check_linux.py

Checks everything that differs between the Windows PC and a Linux box, and
prints ONE line per item: ok / !! (must fix) / -- (optional). Nothing moves,
nothing is written, no robot command is sent.
"""
import importlib
import os
import platform
import socket
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
problems = 0


def line(ok, what, fix="", optional=False):
    global problems
    if ok:
        print(f"  ok  {what}")
    elif optional:
        print(f"  --  {what}" + (f"   ({fix})" if fix else ""))
    else:
        problems += 1
        print(f"  !!  {what}" + (f"\n        -> {fix}" if fix else ""))


def can_import(name):
    try:
        importlib.import_module(name)
        return True, ""
    except Exception as e:                       # ImportError, OSError from .so
        return False, str(e)


print(f"\n  {platform.system()} {platform.machine()}, Python {platform.python_version()}\n")
arm = platform.machine() in ("aarch64", "arm64")

# ---- python
line(sys.version_info >= (3, 10), "Python 3.10 or newer",
     "telegram_ask.py and async_tcp_client.py use 3.10 syntax")

# ---- pip packages
for mod, pkg in (("numpy", "numpy"), ("cv2", "opencv-python"),
                 ("httpx", "httpx"), ("dotenv", "python-dotenv"),
                 ("telegram", "python-telegram-bot"), ("pyodbc", "pyodbc")):
    ok, err = can_import(mod)
    line(ok, f"import {mod}", f"pip install {pkg}   {err}")

ok, err = can_import("pyrealsense2")
line(ok, "import pyrealsense2", f"pip install pyrealsense2   {err}")

# ---- Jetson
tegra = "/etc/nv_tegra_release"
if os.path.exists(tegra):
    print(f"      Jetson: {open(tegra).readline().strip()[:60]}")

ok, err = can_import("torch")
if ok:
    import torch
    cuda = torch.cuda.is_available()
    line(cuda, f"torch {torch.__version__} sees the GPU",
         "this torch came from pypi, not NVIDIA -- pip uninstall torch torchvision "
         "and redo LINUX_SETUP.md step 3" if arm else
         "no CUDA: the model will run on the CPU", optional=not arm)
else:
    line(False, "import torch",
         "LINUX_SETUP.md step 3 (NVIDIA's wheel) -- BEFORE requirments.txt"
         if arm else "pip install torch", )

ok, err = can_import("ultralytics")
line(ok, "import ultralytics (the AI model)", "pip install -r requirments.txt")

if arm:
    try:
        import numpy
        line(numpy.__version__.startswith("1."), f"numpy {numpy.__version__} < 2 (Jetson torch)",
             "pip install 'numpy<2' 'opencv-python<4.12'")
    except Exception:
        pass

# ---- modules of this project
ok, err = can_import("logstore")
line(ok, "logstore.py (client.py needs it)", err)
ok, err = can_import("cobot_kit")
line(ok, "cobot_kit imports", err)

# ---- Fairino SDK
ok, err = can_import("fairino.Robot")
if ok:
    line(True, "fairino SDK imports")
else:
    folder = os.path.join(HERE, "fairino")
    files = os.listdir(folder) if os.path.isdir(folder) else []
    pyd = [f for f in files if f.endswith(".pyd")]
    wrong_so = [f for f in files if f.endswith(".so")
                and platform.machine() not in f]
    if not files:
        fix = f"the fairino/ folder (with Robot.py) is missing from the project   ({err})"
    elif pyd:
        fix = ("a Windows .pyd sits directly in fairino/ -- delete it; "
               "fairino/Robot.py is all that is needed")
    elif wrong_so:
        fix = (f"compiled for another CPU ({wrong_so[0]}) -- this box is "
               f"{platform.machine()}, Python {sys.version_info[0]}.{sys.version_info[1]}")
    else:
        fix = err
    line(False, "fairino SDK", fix)

# ---- ODBC
try:
    import pyodbc
    drivers = pyodbc.drivers()
    good = [d for d in drivers if "ODBC Driver 1" in d]
    line(bool(good), f"ODBC driver for SQL Server ({', '.join(drivers) or 'none'})",
         "sudo ACCEPT_EULA=Y apt install msodbcsql18 (LINUX_SETUP.md)")
except Exception as e:
    line(False, "ODBC driver manager", f"sudo apt install unixodbc   ({e})")

for i in (1, 2):
    f = os.path.join(HERE, f"last_db{i}_settings.txt")
    if os.path.exists(f):
        auth = open(f, encoding="utf-8-sig").read().split("|")
        line(not (len(auth) == 5 and auth[2] == "Windows Authentication"),
             f"last_db{i}_settings.txt uses SQL authentication",
             "Windows Authentication does not work on Linux without Kerberos -- "
             "use a SQL user/password")
    else:
        line(False, f"last_db{i}_settings.txt", "copy it from the Windows PC",
             optional=True)

# ---- telegram
line(os.path.exists(os.path.join(HERE, ".env")), ".env with BOT_TOKEN",
     "copy .env from the Windows PC (telegram_ask.ask() needs it)", optional=True)

# ---- camera
try:
    import pyrealsense2 as rs
    n = len(rs.context().query_devices())
    line(n > 0, f"RealSense cameras visible: {n}",
         "plug it into a USB 3 port; if lsusb shows it but this is 0, "
         "install the udev rules (LINUX_SETUP.md)")
except Exception:
    pass

# ---- screen (ClickPicker / cv2.imshow)
line(bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")),
     "a screen (DISPLAY) for ClickPicker / cv2.imshow",
     "not needed when the AI picks the points", optional=True)

# ---- robot network
try:
    from cobot_kit import settings
except Exception:
    settings = None
if settings is not None:
    ip = settings.ROBOT.get("ip")
    try:
        socket.create_connection((ip, 20003), timeout=2).close()
        line(True, f"Fairino controller reachable at {ip}")
    except Exception as e:
        line(False, f"Fairino controller at {ip}",
             f"give this PC a static IP on the robot's subnet (nmcli / netplan)  ({e})",
             optional=True)

# ---- weights
try:
    w = settings.AI.get("weights")
    here = [p for p in (os.path.join(HERE, w), os.path.join(HERE, "cobot_kit", w))
            if os.path.exists(p)]
    line(bool(here), f"model weights '{w}'",
         "Linux file names are CASE-SENSITIVE: Weld.pt != weld.pt", optional=True)
except Exception:
    pass

print(f"\n  {'READY' if not problems else f'{problems} thing(s) to fix'}\n")
sys.exit(1 if problems else 0)
