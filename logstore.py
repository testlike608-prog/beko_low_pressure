"""
logstore -- a permanent copy of every log line, on disk.

client.py and async_tcp_client.py import this and call:

    logstore.write(name, level, msg)

It was missing from the project, so `import client` failed. This version
writes one file per client name under logs/ beside this file, rotates at
5 MB (keeps 5 old files), and NEVER raises -- a full disk or a read-only
folder must not take the TCP client down with it.
"""
import logging
import os
import re
import threading
from logging.handlers import RotatingFileHandler

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
MAX_BYTES = 5 * 1024 * 1024
BACKUPS = 5

_lock = threading.Lock()
_loggers = {}
_LEVELS = {"DEBUG": logging.DEBUG, "INFO": logging.INFO,
           "WARNING": logging.WARNING, "ERROR": logging.ERROR,
           "CRITICAL": logging.CRITICAL}


def _logger(name):
    safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", name) or "unnamed"
    with _lock:
        lg = _loggers.get(safe)
        if lg is None:
            os.makedirs(LOG_DIR, exist_ok=True)
            lg = logging.getLogger(f"logstore.{safe}")
            lg.setLevel(logging.DEBUG)
            lg.propagate = False            # the client already prints to the console
            h = RotatingFileHandler(os.path.join(LOG_DIR, f"{safe}.log"),
                                    maxBytes=MAX_BYTES, backupCount=BACKUPS,
                                    encoding="utf-8")
            h.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
            lg.addHandler(h)
            _loggers[safe] = lg
        return lg


def write(name, level, msg):
    try:
        _logger(str(name)).log(_LEVELS.get(str(level).upper(), logging.INFO), str(msg))
    except Exception:
        pass
