"""
camera_hub.py
=============
Module موحد لكل أنواع الكاميرات — نفس الـ pattern بتاع ai_vision.py.

API:
    cam = CameraHub.OpenCV(camera_index=0)         ← ويب كام / USB عادي
    cam = CameraHub.UseePlus(camera_index=0)       ← useeplus endoscope (VID=0x2CE3)
    cam = CameraHub.RealSense(camera_index=0)      ← Intel RealSense D4xx (colour+depth)

    cam.start()          → يبدأ في ثريد خلفي
    cam.stop()           → يوقف
    cam.restart()        → يوقف ويعيد التشغيل
    cam.get_frame()      → numpy array BGR أو None
    cam.is_running()     → True لو شغال
    cam.wait_for_frame() → ينتظر أول فريم

زيادة في الـ depth cameras بس (RealSense):
    cam.get_frames()     → (color, depth) من نفس اللحظة
    cam.depth_at(u, v)   → العمق بالمتر عند بكسل، أو None
    cam.deproject(u, v)  → (x, y, z) بالمليمتر في الـ camera frame
    cam.intrinsics       → fx / fy / cx / cy مقروءة من الجهاز

إضافة camera type جديد (3 خطوات):
    1. اعمل class يورث من CameraHub
    2. نفذ _capture_loop(camera_index) فقط — الباقي جاهز
    3. اربطه:  CameraHub.MyCamera = MyCameraClass
"""

from __future__ import annotations

import threading
import time
import logging
from abc import ABC, abstractmethod

log = logging.getLogger("camera_hub")


# ══════════════════════════════════════════════════════════════════════════════
#  PARENT CLASS
# ══════════════════════════════════════════════════════════════════════════════

class CameraHub(ABC):
    """
    الكلاس الأب المشترك لكل camera drivers.

    بيوفر:
      - State management  : thread، locks، latest frame
      - start() / stop()  : lifecycle كامل مع thread safety
      - restart()         : stop + start بـ camera_index جديد
      - get_frame()       : يرجع نسخة من آخر فريم بأمان
      - is_running()      : حالة الـ thread
      - wait_for_frame()  : ينتظر أول فريم (مفيد بعد start)

    _capture_loop() هو الـ abstract الوحيد — كل driver بينفذه بنفسه.

    Interfaces:
      CameraHub.OpenCV    — cv2.VideoCapture (ويب كام / USB عادي)
      CameraHub.UseePlus  — useeplus USB endoscope (VID=0x2CE3 / PID=0x3828)
      CameraHub.RealSense — Intel RealSense D4xx (colour + depth + intrinsics)
    """

    DEFAULT_CAM_INDEX = 0

    def __init__(
        self,
        camera_index: int | None = None,
        frame_width: int = 1280,
        frame_height: int = 720,
    ):
        """
        Parameters
        ----------
        camera_index : رقم الكاميرا الافتراضي (ممكن يتغير في start/restart)
        frame_width  : العرض المطلوب (بيُطبَّق لو الـ driver يدعمه)
        frame_height : الارتفاع المطلوب
        """
        self._cam_index    = camera_index if camera_index is not None else self.DEFAULT_CAM_INDEX
        self.frame_width   = frame_width
        self.frame_height  = frame_height

        self._stop_event   = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock         = threading.Lock()   # يحمي _thread
        self._frame_lock   = threading.Lock()   # يحمي _latest_frame
        self._latest_frame = None               # آخر فريم (numpy array BGR)
        self._frame_seq    = 0                  # بيزيد مع كل فريم جديد

    # ── Public API ────────────────────────────────────────────────────────────

    def get_frame(self):
        """
        يرجع نسخة (copy) من آخر فريم أو None لو مفيش فريم بعد.
        آمن للاستخدام من أي thread.
        """
        with self._frame_lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    def is_running(self) -> bool:
        """يرجع True لو الـ capture loop شغال."""
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    def wait_for_frame(self, timeout: float = 5.0) -> bool:
        """
        يستنى لحد ما أول فريم يتقرأ (أو timeout).
        يرجع True لو جه الفريم، False لو انتهى الوقت بدون فريم.
        استخدمه دايماً بعد start() وقبل ما تبدأ تقرأ فريمات.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._frame_lock:
                if self._latest_frame is not None:
                    log.info(f"{self._log_name}: ✓ أول فريم اتقرأ")
                    return True
            time.sleep(0.05)
        log.error(f"{self._log_name}: ✗ timeout {timeout}s — مفيش فريم!")
        return False

    @property
    def frame_seq(self) -> int:
        """
        رقم بيزيد مع كل فريم جديد.

        ليه موجود: الكاميرا هنا push — بتصوّر على طول في الخلفية — فـ
        get_frame() ممكن تديك صورة اتاخدت **قبل** ما الذراع توصل
        مكانها. وده فرق مبيبانش — الصورة سليمة وواضحة، بس من مكان تاني.
        مقارنة الرقم ده قبل وبعد هي اللي بتقول لك هل الفريم جديد ولا لأ.
        """
        with self._frame_lock:
            return self._frame_seq

    def wait_for_new_frame(self, since: int, timeout: float = 5.0) -> bool:
        """
        يستنى فريم **أحدث من** رقم معين (من frame_seq).

        يرجع True لو جه فريم جديد، False لو الوقت خلص أو الكاميرا وقفت.
        بعدها نادي get_frame() / get_frames() عادي.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._frame_lock:
                if self._frame_seq > since and self._latest_frame is not None:
                    return True
            if not self.is_running():
                return False
            time.sleep(0.005)
        return False

    def start(self, camera_index: int | None = None):
        """
        يبدأ التقاط الفريمات في ثريد خلفي.
        لو شغالة بالفعل مش بيعمل حاجة.
        """
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                log.debug(f"{self._log_name}: start() — شغالة بالفعل")
                return

            if camera_index is not None:
                self._cam_index = camera_index
            elif self._cam_index is None:
                try:
                    from config import config as _cfg
                    self._cam_index = int(_cfg.get("camera_index", self.DEFAULT_CAM_INDEX))
                except Exception:
                    self._cam_index = self.DEFAULT_CAM_INDEX

            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._capture_loop,
                args=(self._cam_index,),
                name=self._thread_name,
                daemon=True,
            )
            self._thread.start()
            log.info(f"{self._log_name}: بدأت (كاميرا {self._cam_index})")

    def stop(self, timeout: float = 3.0):
        """يوقف الكاميرا وينتظر الثريد ينتهي."""
        with self._lock:
            if self._thread is None or not self._thread.is_alive():
                log.debug(f"{self._log_name}: stop() — مش شغالة")
                return
            self._stop_event.set()
            t = self._thread

        t.join(timeout=timeout)
        if t.is_alive():
            log.warning(f"{self._log_name}: الثريد لم ينتهِ في الوقت المحدد")
        else:
            log.info(f"{self._log_name}: أوقفت بنجاح")

        with self._lock:
            self._thread = None

    def restart(self, camera_index: int | None = None) -> bool:
        """
        يوقف الكاميرا ويشغّلها تاني برقم جديد (اختياري).
        يرجع True لو نجح وجه أول فريم، False لو فشل.
        """
        idx = camera_index or self._cam_index
        log.info(f"{self._log_name}: restarting (camera {idx})...")
        self.stop(timeout=3.0)
        time.sleep(0.2)  # استنى الـ driver يحرر الكاميرا
        self.start(camera_index=idx)
        ok = self.wait_for_frame(timeout=6.0)
        if ok:
            log.info(f"{self._log_name}: restarted successfully (camera {idx})")
        else:
            log.error(f"{self._log_name}: restart failed — camera {idx} لم تستجب")
        return ok

    # ── Internal helpers ──────────────────────────────────────────────────────

    @property
    def _log_name(self) -> str:
        return self.__class__.__name__

    @property
    def _thread_name(self) -> str:
        return f"camera-{self.__class__.__name__.lower()}"

    def _set_frame(self, frame):
        """يحدّث الـ latest frame بشكل آمن (للاستخدام داخل _capture_loop)."""
        with self._frame_lock:
            self._latest_frame = frame
            self._frame_seq   += 1

    def _clear_frame(self):
        """يمسح الـ frame الأخير (عند الإغلاق)."""
        with self._frame_lock:
            self._latest_frame = None

    # ── Abstract ──────────────────────────────────────────────────────────────

    @abstractmethod
    def _capture_loop(self, camera_index: int):
        """
        الـ loop الأساسي للكاميرا — يشتغل في ثريد خلفي.

        لازم:
          - يستخدم self._set_frame(frame) لتحديث الفريم
          - يراقب self._stop_event.is_set() للخروج
          - ينهي بـ self._clear_frame() في finally
        """
        ...


# ══════════════════════════════════════════════════════════════════════════════
#  INTERFACE: OpenCV  (ويب كام / USB عادي)
# ══════════════════════════════════════════════════════════════════════════════

class _OpenCV(CameraHub):
    """
    CameraHub.OpenCV — أي كاميرا بيدعمها OpenCV (ويب كام، USB، RTSP).

    مثال:
        cam = CameraHub.OpenCV(camera_index=0)
        cam.start()
        frame = cam.get_frame()
    """

    def _capture_loop(self, camera_index: int):
        import cv2

        # DSHOW أسرع على Windows، وإلا auto-detect
        cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = cv2.VideoCapture(camera_index)
        if not cap.isOpened():
            log.error(
                f"{self._log_name}: ❌ مش قادر أفتح الكاميرا {camera_index} "
                "— تأكد إنها متوصلة ومش مفتوحة ببرنامج تاني"
            )
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        log.info(f"{self._log_name}: ✅ Camera {camera_index} شغالة ({actual_w}x{actual_h})")
        print(f"[{self._log_name}] opened camera index={camera_index} ({actual_w}x{actual_h})")

        try:
            while not self._stop_event.is_set():
                ret, frame = cap.read()
                if ret:
                    self._set_frame(frame)
                else:
                    log.warning(f"{self._log_name}: فريم فاشل، هحاول تاني...")
                    time.sleep(0.05)
                    continue
                time.sleep(0.01)   # ~100 fps max
        except Exception as e:
            log.error(f"{self._log_name}: خطأ غير متوقع: {e}")
        finally:
            cap.release()
            self._clear_frame()
            log.info(f"{self._log_name}: الكاميرا اتقفلت.")


# ══════════════════════════════════════════════════════════════════════════════
#  INTERFACE: UseePlus  (USB endoscope — VID=0x2CE3 / PID=0x3828)
# ══════════════════════════════════════════════════════════════════════════════

class _UseePlus(CameraHub):
    """
    CameraHub.UseePlus — useeplus SuperCamera (USB endoscope).

    ملاحظات مهمة:
      - السنسور بيبعت 640×480 MJPEG فقط (أقصى دقة هاردوير حقيقية).
      - أبلكيشن UseePlus بيعمل upscale برمجي ×3 → 1920×1440.
        الكلاس بيعمل نفس الحاجة افتراضياً (upscale=True + sharpen=True).
      - البروتوكول (UPP): كل رسالة USB = هيدر 5 بايت (magic + camera_id + length)
        + هيدر كاميرا 7 بايت (frame_id + cam_num + flags + g_sensor) + جزء من الـ JPEG.
        بنجمّع الأجزاء لحد ما الـ frame_id يتغير = فريم كامل.
      - أي فريم هيدرز رسايله متضاربة أو الـ JPEG بتاعه ناقص بيترمي بالكامل —
        عشان كده مش هتشوف فريمات مشوهة أبداً (زي الأبلكيشن بالظبط).

    المتطلبات:
        pip install pyusb opencv-python numpy libusb-package
        + Zadig (WinUSB driver) للكاميرا على Windows

    مثال:
        cam = CameraHub.UseePlus(camera_index=0)   # 1920×1440 زي الأبلكيشن
        cam = CameraHub.UseePlus(upscale=False)    # 640×480 خام
        cam.start()
        frame = cam.get_frame()
    """

    # ── USB Constants ─────────────────────────────────────────────────────────
    VENDOR_ID    = 0x2CE3
    PRODUCT_ID   = 0x3828
    INTERFACE    = 1
    ALT_SETTING  = 1
    EP_OUT       = 0x01
    EP_OUT2      = 0x02
    EP_IN        = 0x81
    MAGIC_WORDS  = bytes([0xFF, 0x55, 0xFF, 0x55, 0xEE, 0x10])  # init على EP2 (الأبلكيشن بيبعتها)
    CONNECT_CMD  = bytes([0xBB, 0xAA, 0x05, 0x00, 0x00])        # start stream

    # ── UPP protocol ──────────────────────────────────────────────────────────
    USB_MAGIC    = bytes([0xAA, 0xBB])  # uint16 LE = 0xBBAA
    USB_HDR_LEN  = 5     # magic(2) + camera_id(1) + length(2 LE، مش شاملة الهيدر)
    CAM_HDR_LEN  = 7     # frame_id(1) + cam_num(1) + flags(1) + g_sensor(4)
    VALID_CIDS   = (7, 11)
    MAX_MSG_LEN  = 4096  # sanity limit للـ length field
    JPEG_SOI     = bytes([0xFF, 0xD8])
    JPEG_EOI     = bytes([0xFF, 0xD9])

    READ_TIMEOUT  = 500       # ms — أقصر عشان نكتشف freeze بسرعة
    WRITE_TIMEOUT = 5000      # ms
    CHUNK_SIZE    = 64 * 1024 # 64 KB per USB read
    FREEZE_TIMEOUT = 2.0      # ثواني بدون فريم → recovery
    STARTUP_GRACE  = 2.0      # قبل أول فريم: إعادة إرسال CONNECT كل ثانيتين
                              # (بعض الكاميرات محتاجة الأمر يتبعت أكتر من مرة)
    BUF_MAX        = 2 * 1024 * 1024  # 2 MB حد أقصى للبفر

    NATIVE_SIZE    = (640, 480)  # دقة السنسور الحقيقية

    def __init__(
        self,
        camera_index: int | None = None,
        frame_width: int = 1920,
        frame_height: int = 1440,
        upscale: bool = True,
        sharpen: bool = True,
        ep2_init: bool = False,
        clear_halt_init: bool = False,
    ):
        """
        upscale         : يكبّر الفريم لـ frame_width×frame_height (زي الأبلكيشن بالظبط)
        sharpen         : unsharp mask خفيف بعد التكبير (نفس مظهر الأبلكيشن)
        ep2_init        : يبعت أمر init على EP2 قبل الستريم —
                          ⚠️ مقفول افتراضياً: ثبت إنه بيكسر الستريم على بعض الأجهزة
        clear_halt_init : يعمل clear_halt + claim للـ interfaces عند الفتح —
                          ⚠️ مقفول افتراضياً: بعض الـ USB controllers بتتلخبط منه
        """
        super().__init__(camera_index, frame_width, frame_height)
        self.upscale = upscale
        self.sharpen = sharpen
        self.ep2_init = ep2_init
        self.clear_halt_init = clear_halt_init
        self.on_button_press = None  # callback اختياري لزرار الإندوسكوب

    def _capture_loop(self, camera_index: int):
        import queue
        import numpy as np
        import cv2

        # ── تحميل pyusb ───────────────────────────────────────────────────────
        try:
            import usb.core
            import usb.util
            import usb.backend.libusb1 as _lb1
        except ImportError:
            log.error(f"{self._log_name}: ❌ pyusb مش متثبّت — شغّل: pip install pyusb")
            return

        # الـ backend: بنجرب libusb-package الأول، وبعدها libusb، وبعدها الافتراضي
        _backend = None
        try:
            import libusb_package
            _backend = _lb1.get_backend(find_library=libusb_package.find_library)
        except Exception:
            pass
        if _backend is None:
            try:
                import libusb
                _backend = _lb1.get_backend(find_library=lambda x: libusb.dll._name)
            except Exception:
                pass
        if _backend is None:
            log.warning(
                f"{self._log_name}: ⚠️ مفيش libusb backend صريح — "
                "هجرب الافتراضي. لو الكاميرا مش لاقيها: pip install libusb-package"
            )

        # ── إيجاد الجهاز ──────────────────────────────────────────────────────
        kw = {"backend": _backend} if _backend else {}
        try:
            devices = list(usb.core.find(
                idVendor=self.VENDOR_ID, idProduct=self.PRODUCT_ID,
                find_all=True, **kw,
            ))
        except Exception as e:
            log.error(
                f"{self._log_name}: ❌ مفيش libusb backend — "
                f"شغّل: pip install libusb-package  ({e})"
            )
            return
        if not devices:
            log.error(
                f"{self._log_name}: ❌ الكاميرا مش موجودة "
                "— تأكد USB متوصل + Zadig مثبّت"
            )
            return

        if camera_index >= len(devices):
            log.warning(
                f"{self._log_name}: camera_index={camera_index} أكبر من "
                f"عدد الأجهزة ({len(devices)}) — هستخدم 0"
            )
            camera_index = 0

        dev = devices[camera_index]

        # ── إعداد USB (نفس تسلسل الأبلكيشن الرسمي) ───────────────────────────
        try:
            for _intf in (0, self.INTERFACE):
                try:
                    if dev.is_kernel_driver_active(_intf):
                        dev.detach_kernel_driver(_intf)
                except Exception:
                    pass
            dev.set_configuration()
            if self.clear_halt_init:
                # claim صريح للـ interfaces — خطوة إضافية زي الأبلكيشن،
                # بتتقفل مع safeinit لأن بعض الأجهزة بتتلخبط منها
                try:
                    usb.util.claim_interface(dev, 0)
                    usb.util.claim_interface(dev, self.INTERFACE)
                except Exception:
                    pass
            dev.set_interface_altsetting(
                interface=self.INTERFACE,
                alternate_setting=self.ALT_SETTING,
            )
            if self.clear_halt_init:
                for _ep in (self.EP_OUT, self.EP_IN):
                    try:
                        dev.clear_halt(_ep)
                    except Exception:
                        pass
            # "الكلمات السحرية" — الأبلكيشن بيبعتها على EP2 قبل بدء الستريم
            if self.ep2_init:
                try:
                    dev.write(self.EP_OUT2, self.MAGIC_WORDS, self.WRITE_TIMEOUT)
                except Exception as e:
                    log.warning(f"{self._log_name}: ⚠️ EP2 init فشل (غالباً مش مشكلة): {e}")
            dev.write(self.EP_OUT, self.CONNECT_CMD, self.WRITE_TIMEOUT)
            log.info(
                f"{self._log_name}: ✅ Camera {camera_index} شغالة "
                f"(VID={self.VENDOR_ID:#06x} PID={self.PRODUCT_ID:#06x})"
            )
            print(f"[{self._log_name}] opened useeplus camera index={camera_index}")
        except Exception as e:
            log.error(f"{self._log_name}: ❌ فشل تهيئة USB: {e}")
            return

        # ── Decode thread — منفصل لتجنب blocking ─────────────────────────────
        decode_q: queue.Queue = queue.Queue(maxsize=3)
        out_size = (self.frame_width, self.frame_height)

        def _decode_worker():
            while True:
                item = decode_q.get()
                if item is None:
                    break
                arr     = np.frombuffer(item, dtype=np.uint8)
                decoded = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if decoded is not None:
                    if self.upscale and (decoded.shape[1], decoded.shape[0]) != out_size:
                        # نفس اللي الأبلكيشن بيعمله: upscale + شوية sharpening
                        decoded = cv2.resize(
                            decoded, out_size, interpolation=cv2.INTER_LANCZOS4
                        )
                        if self.sharpen:
                            blur    = cv2.GaussianBlur(decoded, (0, 0), 1.5)
                            decoded = cv2.addWeighted(decoded, 1.4, blur, -0.4, 0)
                    self._set_frame(decoded)
                decode_q.task_done()

        dec_thread = threading.Thread(
            target=_decode_worker, name="cam-useeplus-decode", daemon=True
        )
        dec_thread.start()

        # ── UPP stream parser ─────────────────────────────────────────────────
        buf       = bytearray()  # بيانات USB خام
        frame_buf = bytearray()  # payload الفريم الحالي
        cur_hdr   = None         # (fid, cam_num, has_g, other) بتوع الفريم الحالي
        frame_bad = False
        stats     = {"ok": 0, "dropped": 0}

        def _reset_frame():
            nonlocal cur_hdr, frame_bad
            frame_buf.clear()
            cur_hdr   = None
            frame_bad = False

        def _finish_frame() -> bytes | None:
            """يقفل الفريم الحالي — يرجع JPEG سليم أو None (فريم بايظ = يترمي)."""
            jpeg = None
            if not frame_bad and frame_buf[:2] == self.JPEG_SOI:
                eoi = frame_buf.rfind(self.JPEG_EOI)
                if eoi != -1:
                    jpeg = bytes(frame_buf[:eoi + 2])
            if jpeg:
                stats["ok"] += 1
            else:
                stats["dropped"] += 1
                log.debug(
                    f"{self._log_name}: فريم مرفوض "
                    f"(bad={frame_bad}, len={len(frame_buf)})"
                )
            _reset_frame()
            return jpeg

        def _parse_stream() -> list:
            """
            يفكك رسائل UPP من buf ويرجع الفريمات الكاملة السليمة فقط.
            رسالة = [AA BB][cid][len LE][fid][cam][flags][g_sensor x4][payload]
            """
            nonlocal cur_hdr, frame_bad
            frames = []

            while True:
                if len(buf) < self.USB_HDR_LEN:
                    break

                # resync: الماجيك لازم يبقى في أول البفر
                if buf[:2] != self.USB_MAGIC:
                    idx = buf.find(self.USB_MAGIC, 1)
                    if idx == -1:
                        del buf[:-1]  # سيب آخر بايت (الماجيك ممكن يكون متقسم)
                        break
                    del buf[:idx]
                    continue

                cid    = buf[2]
                length = buf[3] | (buf[4] << 8)
                if cid not in self.VALID_CIDS or \
                        not (self.CAM_HDR_LEN <= length <= self.MAX_MSG_LEN):
                    del buf[:2]  # هيدر بايظ → دور على الماجيك اللي بعده
                    continue

                total = self.USB_HDR_LEN + length
                if len(buf) < total:
                    break  # الرسالة لسه ما كملتش — استنى بيانات أكتر

                fid     = buf[5]
                cam_num = buf[6]
                flags   = buf[7]
                has_g   = flags & 0x01
                button  = (flags >> 1) & 0x01
                other   = flags >> 2
                payload = bytes(buf[self.USB_HDR_LEN + self.CAM_HDR_LEN:total])
                del buf[:total]

                if button and self.on_button_press:
                    try:
                        self.on_button_press()
                    except Exception:
                        pass

                # frame_id اتغير = الفريم اللي فات اكتمل
                if cur_hdr is not None and fid != cur_hdr[0]:
                    jpeg = _finish_frame()
                    if jpeg:
                        frames.append(jpeg)

                if cur_hdr is None:
                    # أول رسالة في فريم جديد — الهيدر لازم يكون سليم
                    if cam_num < 2 and has_g == 0 and other == 0:
                        cur_hdr = (fid, cam_num, has_g, other)
                        frame_buf.extend(payload)
                    # لو مش سليم: منتصف فريم قديم — نتجاهل لحد بداية فريم نضيف
                else:
                    if (fid, cam_num, has_g, other) != cur_hdr:
                        frame_bad = True  # تضارب → الفريم كله هيترمي
                    else:
                        frame_buf.extend(payload)

            return frames

        def _recover():
            """يصحّح الـ USB endpoint بعد pipe error أو freeze."""
            try:
                dev.clear_halt(self.EP_IN)
                if self.ep2_init:
                    try:
                        dev.write(self.EP_OUT2, self.MAGIC_WORDS, self.WRITE_TIMEOUT)
                    except Exception:
                        pass
                dev.write(self.EP_OUT, self.CONNECT_CMD, self.WRITE_TIMEOUT)
                buf.clear()
                _reset_frame()
                log.info(f"{self._log_name}: ✅ endpoint cleared — stream restarted")
            except Exception as e:
                log.warning(f"{self._log_name}: ⚠️ recovery failed: {e}")

        # ── Read loop ─────────────────────────────────────────────────────────
        last_frame_time = time.time()
        got_first_frame = False
        recovery_count  = 0

        try:
            while not self._stop_event.is_set():
                try:
                    raw = bytes(dev.read(self.EP_IN, self.CHUNK_SIZE, self.READ_TIMEOUT))
                except Exception as e:
                    err = str(e).lower()

                    if "timed out" in err:
                        # قبل أول فريم بنستنى أطول — عشان recovery مايقطعش
                        # الستريم والكاميرا لسه بتثبّت (ده كان سبب التقطيع في الأول)
                        limit = self.FREEZE_TIMEOUT if got_first_frame \
                                else self.STARTUP_GRACE
                        if time.time() - last_frame_time > limit:
                            recovery_count += 1
                            log.warning(
                                f"{self._log_name}: ⚠️ freeze #{recovery_count} — recovering..."
                            )
                            _recover()
                            last_frame_time = time.time()
                        continue

                    if "pipe" in err or "errno 32" in err or "stall" in err:
                        recovery_count += 1
                        log.warning(
                            f"{self._log_name}: ⚠️ pipe error #{recovery_count} — recovering..."
                        )
                        _recover()
                        last_frame_time = time.time()
                        time.sleep(0.05)
                        continue

                    log.error(f"{self._log_name}: خطأ USB read: {e}")
                    time.sleep(0.1)
                    continue

                if not raw:
                    continue

                buf.extend(raw)

                # حماية — مفروض مايحصلش مع البارسر الجديد
                if len(buf) > self.BUF_MAX:
                    buf.clear()
                    _reset_frame()

                for jpeg in _parse_stream():
                    got_first_frame = True
                    last_frame_time = time.time()
                    try:
                        decode_q.put_nowait(jpeg)
                    except queue.Full:
                        # ارمي الأقدم وحافظ على الأحدث (مش العكس!)
                        try:
                            decode_q.get_nowait()
                        except queue.Empty:
                            pass
                        try:
                            decode_q.put_nowait(jpeg)
                        except queue.Full:
                            pass

        except Exception as e:
            log.error(f"{self._log_name}: خطأ غير متوقع: {e}")
        finally:
            decode_q.put(None)
            dec_thread.join(timeout=2.0)
            try:
                dev.set_interface_altsetting(
                    interface=self.INTERFACE, alternate_setting=0
                )
                usb.util.dispose_resources(dev)
            except Exception:
                pass
            self._clear_frame()
            log.info(
                f"{self._log_name}: الكاميرا اتقفلت. "
                f"(frames ok={stats['ok']} dropped={stats['dropped']} "
                f"recoveries={recovery_count})"
            )


# ══════════════════════════════════════════════════════════════════════════════
#  INTERFACE: RealSense  (Intel D4xx — colour + depth)
# ══════════════════════════════════════════════════════════════════════════════

class _RealSense(CameraHub):
    """
    CameraHub.RealSense — كاميرا Intel RealSense D4xx (D435 / D435i / D455).

    الفرق الوحيد عن باقي الـ interfaces إنها بتدي **depth** كمان، وده بيفتح
    حاجات مش موجودة في كاميرا 2D:

        cam.get_frame()      → colour BGR (زي أي كاميرا تانية — نفس الـ API)
        cam.get_depth()      → depth بالمتر (float32) أو None
        cam.get_frames()     → (color, depth) الاتنين من نفس اللحظة بالظبط
        cam.depth_at(u, v)   → العمق بالمتر عند بكسل، أو None
        cam.deproject(u, v)  → (x, y, z) بالمليمتر في الـ camera frame
        cam.intrinsics       → dict فيه fx / fy / cx / cy مقروءة من الجهاز

    ست حاجات متغطية هنا، كل واحدة فيهم بتضيع يوم لو مش متعملة:

    1. **USB 2.1 بيحدّد الدقة.** الـ D435i على USB 2 مبيقدرش يطلّع 1280×720
       غير على 6 fps؛ لو طلبت 30 بيرفض بـ error مالوش أي علاقة بالموضوع.
       عشان كده فيه `FALLBACKS` — لو الـ profile المطلوب اترفض بيجرب اللي بعده
       وبيقول لك في اللوج على أنهي profile استقر فعلاً.
    2. **الـ depth لازم يتعمله align على الـ colour**، وإلا البكسل اللي الموديل
       شايفه مش هو البكسل اللي العمق بتاعه — والفرق بيوصل سنتيمترات على الحرف.
       مفعّل افتراضياً (`align=True`).
    3. **الـ depth scale بيتقرا من الجهاز**، مش 0.001 مكتوبة بإيدك. فيه موديلات
       بتديك غير كده، والفرق بيطلع كأنه خطأ في الـ calibration.
    4. **الصفر في الـ depth معناه "مفيش قراءة"** — لحام لامع، أو قريب أوي، أو
       في ظل — مش "على اللنس". `depth_at()` بترمي الأصفار وبتاخد median على
       patch بدل بكسل واحد، لأن بكسل واحد على حرف بيقع على الخلفية ويديك
       نص متر غلط من غير ما حد ياخد باله.
    5. **`wait_for_frames` بيرمي RuntimeError عند الـ timeout** — دي حالة عادية
       بتتعاد، مش سبب تقفل الكاميرا. لكن لو اتكررت بيحذّرك.
    6. **الـ intrinsics بتتقرا من الـ profile الشغال**، بعد الـ align — مش من
       الداتاشيت ولا من اللي إنت طلبته، لأن الكاميرا ممكن تديك profile تاني.

    المتطلبات:
        pip install pyrealsense2 opencv-python numpy

    مثال:
        cam = CameraHub.RealSense(camera_index=0)          # 1280×720 @ 6 (USB 2.1)
        cam = CameraHub.RealSense(frame_width=848, frame_height=480, fps=15)
        cam = CameraHub.RealSense(serial="123456789012")   # كاميرا بعينها
        cam = CameraHub.RealSense(depth=False)             # colour بس، أسرع

        cam.start()
        cam.wait_for_frame()
        color, depth = cam.get_frames()
        xyz_mm = cam.deproject(640, 360)
    """

    #: (width, height, fps) — بيتجربوا بالترتيب لو اللي إنت طلبته اترفض.
    #: مرتبين من الأعلى للأقل عشان تخسر أقل حاجة ممكنة.
    FALLBACKS = ((1280, 720, 6), (848, 480, 15), (640, 480, 15), (640, 480, 6))

    #: أقصر من الافتراضي عن قصد: الـ loop بيبص على _stop_event كل ثانية،
    #: فـ stop() بترجع في وقتها بدل ما تستنى الـ frame timeout يخلص
    FRAME_TIMEOUT_MS        = 1000
    TIMEOUTS_BEFORE_WARNING = 5

    DEPTH_PATCH = 5    # median على 5×5 — مش بكسل واحد

    def __init__(
        self,
        camera_index: int | None = None,
        frame_width: int = 1280,
        frame_height: int = 720,
        fps: int = 6,
        depth: bool = True,
        align: bool = True,
        serial: str | None = None,
    ):
        """
        fps    : 6 افتراضي — ده اللي بيشتغل على USB 2.1 بدقة 1280×720
        depth  : False = colour بس (أسرع، وبيشيل حمل الـ align)
        align  : يعمل align للـ depth على الـ colour — سيبه True
        serial : سيريال كاميرا بعينها؛ لو محطوطة بتتجاهل camera_index
        """
        super().__init__(camera_index, frame_width, frame_height)
        self.fps             = fps
        self.want_depth      = depth
        self.align_to_color  = align
        self.serial          = serial

        self._latest_depth   = None    # float32 metres, aligned to colour
        self._depth_scale    = None    # بيتقرا من الجهاز
        self._intrinsics     = None    # dict، متاح بعد ما الستريم يشتغل
        self._profile_used   = None    # (w, h, fps) اللي استقر فعلاً

    # ── depth-side API (الجزء اللي مش موجود في كاميرا 2D) ────────────────────

    def get_depth(self):
        """آخر depth frame بالمتر (float32) أو None. آمن من أي thread."""
        with self._frame_lock:
            return self._latest_depth.copy() if self._latest_depth is not None else None

    def get_frames(self):
        """
        (color, depth) — الاتنين من **نفس اللحظة**.

        استخدم دي مش get_frame() + get_depth() ورا بعض: النداءين المنفصلين ممكن
        يقعوا على جنبين مختلفين من تحديث، فتطلع بلون فريم وعمق فريم اللي بعده.
        """
        with self._frame_lock:
            c = self._latest_frame.copy() if self._latest_frame is not None else None
            d = self._latest_depth.copy() if self._latest_depth is not None else None
        return c, d

    @property
    def intrinsics(self) -> dict | None:
        """fx / fy / cx / cy مقروءة من الجهاز. None قبل ما الستريم يشتغل."""
        return self._intrinsics

    @property
    def depth_scale(self) -> float | None:
        """وحدة الـ depth بتاعة الجهاز بالمتر. None قبل التشغيل."""
        return self._depth_scale

    @property
    def profile(self) -> tuple | None:
        """(w, h, fps) اللي الكاميرا استقرت عليه فعلاً — مش اللي إنت طلبته."""
        return self._profile_used

    def depth_at(self, u, v, depth=None, patch: int | None = None):
        """
        العمق بالمتر عند بكسل، أو None لو مفيش قراءة تستاهل الثقة.

        الأصفار في صورة الـ depth معناها "مفيش قراءة" مش "على اللنس"، فلو
        حسبناها في المتوسط بتسحب النقطة السليمة ناحية الكاميرا بقد ما فيه
        جيران فاشلين — وده بالظبط نوع الخطأ اللي بيبان بعد أسبوع.

        None رد طبيعي على لحام لامع — نقطة تتعدّى وتتقال، مش cycle يقع.
        """
        import numpy as np

        d = depth if depth is not None else self.get_depth()
        if d is None:
            return None
        h, w = d.shape[:2]
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < w and 0 <= vi < h):
            return None

        r = max(0, (self.DEPTH_PATCH if patch is None else patch) // 2)
        window = d[max(0, vi - r):vi + r + 1, max(0, ui - r):ui + r + 1]
        good = window[window > 0]
        if good.size < max(3, (2 * r + 1) ** 2 // 4):
            return None       # أغلب الـ patch فاضي → مفيش رد نثق فيه
        return float(np.median(good))

    def deproject(self, u, v, depth_m: float | None = None):
        """
        بكسل + عمق → (x, y, z) **بالمليمتر** في الـ camera frame، أو None.

        دي آخر حاجة الكاميرا مسؤولة عنها. اللي بعدها (base frame) شغل الـ
        hand-eye transform، ومش من شغل الملف ده.
        """
        k = self._intrinsics
        if k is None:
            log.warning(f"{self._log_name}: deproject قبل ما الستريم يشتغل")
            return None
        z = self.depth_at(u, v) if depth_m is None else depth_m
        if z is None or z <= 0:
            return None
        mm = float(z) * 1000.0
        return ((float(u) - k["cx"]) / k["fx"] * mm,
                (float(v) - k["cy"]) / k["fy"] * mm,
                mm)

    # ── internal ─────────────────────────────────────────────────────────────

    def _set_pair(self, color, depth):
        """اللون والعمق تحت قفل واحد — عشان get_frames() تطلع متسقة."""
        with self._frame_lock:
            self._latest_frame = color
            self._latest_depth = depth
            self._frame_seq   += 1

    def _clear_frame(self):
        with self._frame_lock:
            self._latest_frame = None
            self._latest_depth = None

    def _capture_loop(self, camera_index: int):
        import numpy as np

        try:
            import pyrealsense2 as rs
        except ImportError:
            log.error(
                f"{self._log_name}: ❌ pyrealsense2 مش متثبّت "
                "— شغّل: pip install pyrealsense2"
            )
            return

        # ── إيجاد الجهاز ──────────────────────────────────────────────────────
        try:
            devices = list(rs.context().query_devices())
        except Exception as e:
            log.error(f"{self._log_name}: ❌ مش قادر أسأل عن أجهزة RealSense: {e}")
            return

        if not devices:
            log.error(
                f"{self._log_name}: ❌ مفيش كاميرا RealSense متوصلة "
                "— جرب RealSense Viewer الأول"
            )
            return

        dev = None
        if self.serial:
            for d in devices:
                try:
                    if d.get_info(rs.camera_info.serial_number) == self.serial:
                        dev = d
                        break
                except Exception:
                    continue
            if dev is None:
                log.error(
                    f"{self._log_name}: ❌ مفيش كاميرا بالسيريال {self.serial} "
                    f"(الموجود: {len(devices)} كاميرا)"
                )
                return
        else:
            if camera_index >= len(devices):
                log.warning(
                    f"{self._log_name}: camera_index={camera_index} أكبر من "
                    f"عدد الأجهزة ({len(devices)}) — هستخدم 0"
                )
                camera_index = 0
            dev = devices[camera_index]

        def _info(field, default="?"):
            try:
                return dev.get_info(getattr(rs.camera_info, field))
            except Exception:
                return default

        serial   = _info("serial_number")
        usb_type = _info("usb_type_descriptor")
        log.info(
            f"{self._log_name}: لقيت {_info('name', 'RealSense')} "
            f"serial={serial} usb={usb_type}"
        )
        if str(usb_type).startswith("2"):
            log.warning(
                f"{self._log_name}: ⚠️ الكاميرا على USB {usb_type} — "
                "الـ bandwidth محدود، متتفاجئش لو الـ fps نزل. "
                "كابل/بورت USB 3 بيحل ده."
            )

        # ── تشغيل الـ pipeline (مع fallbacks) ─────────────────────────────────
        wanted = (self.frame_width, self.frame_height, self.fps)
        attempts = [wanted] + [p for p in self.FALLBACKS if p != wanted]

        pipeline = profile = None
        last_error = None
        for (w, h, fps) in attempts:
            cfg = rs.config()
            try:
                cfg.enable_device(serial)
            except Exception:
                pass
            cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
            if self.want_depth:
                cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)

            pipeline = rs.pipeline()
            try:
                profile = pipeline.start(cfg)
                self._profile_used = (w, h, fps)
                if (w, h, fps) != wanted:
                    log.warning(
                        f"{self._log_name}: ⚠️ {wanted[0]}x{wanted[1]}@{wanted[2]} "
                        f"اترفض — اشتغلت على {w}x{h}@{fps} بدالها"
                    )
                break
            except Exception as e:
                last_error = e
                profile = None
                try:
                    pipeline.stop()
                except Exception:
                    pass
                log.debug(f"{self._log_name}: {w}x{h}@{fps} اترفض: {e}")

        if profile is None:
            log.error(
                f"{self._log_name}: ❌ مفيش profile اشتغل. آخر خطأ: {last_error}\n"
                "    غالباً: الكاميرا مفتوحة في RealSense Viewer، أو USB 2 "
                "مع fps عالي، أو الفيرموير قديم."
            )
            return

        # ── depth scale + intrinsics — من الجهاز، مش من الداتاشيت ────────────
        if self.want_depth:
            try:
                self._depth_scale = float(
                    profile.get_device().first_depth_sensor().get_depth_scale()
                )
            except Exception as e:
                self._depth_scale = 0.001
                log.warning(
                    f"{self._log_name}: ⚠️ مش قادر أقرا depth scale ({e}) "
                    "— هستخدم 0.001 m/unit"
                )

        try:
            vsp = profile.get_stream(rs.stream.color).as_video_stream_profile()
            i = vsp.get_intrinsics()
            self._intrinsics = {
                "width": i.width, "height": i.height,
                "fx": float(i.fx), "fy": float(i.fy),
                "cx": float(i.ppx), "cy": float(i.ppy),
                "distortion": [float(c) for c in i.coeffs],
                "model": str(i.model),
            }
            log.info(
                f"{self._log_name}: intrinsics {i.width}x{i.height} "
                f"fx {i.fx:.1f} fy {i.fy:.1f} cx {i.ppx:.1f} cy {i.ppy:.1f}"
            )
        except Exception as e:
            log.warning(f"{self._log_name}: ⚠️ مش قادر أقرا الـ intrinsics: {e}")

        aligner = None
        if self.want_depth and self.align_to_color:
            try:
                aligner = rs.align(rs.stream.color)
            except Exception as e:
                log.warning(
                    f"{self._log_name}: ⚠️ مش قادر أعمل align ({e}) — "
                    "الـ depth مش هيطابق الـ colour بكسل ببكسل!"
                )

        w, h, fps = self._profile_used
        log.info(
            f"{self._log_name}: ✅ Camera {camera_index} شغالة "
            f"({w}x{h}@{fps}{', depth' if self.want_depth else ''}"
            f"{', aligned' if aligner else ''})"
        )
        print(f"[{self._log_name}] opened realsense {serial} ({w}x{h}@{fps})")

        # ── Read loop ─────────────────────────────────────────────────────────
        consecutive_timeouts = 0
        frames_ok = 0

        try:
            while not self._stop_event.is_set():
                try:
                    fs = pipeline.wait_for_frames(self.FRAME_TIMEOUT_MS)
                except RuntimeError as e:
                    # timeout = حالة عادية بتتعاد، مش سبب نقفل الكاميرا
                    consecutive_timeouts += 1
                    if consecutive_timeouts == self.TIMEOUTS_BEFORE_WARNING:
                        log.warning(
                            f"{self._log_name}: ⚠️ مفيش فريمات من "
                            f"{consecutive_timeouts} محاولة ({e})"
                        )
                    if "disconnect" in str(e).lower():
                        log.error(f"{self._log_name}: ❌ الكاميرا اتفصلت")
                        break
                    continue
                except Exception as e:
                    log.error(f"{self._log_name}: خطأ غير متوقع في القراءة: {e}")
                    time.sleep(0.1)
                    continue

                consecutive_timeouts = 0

                if aligner is not None:
                    try:
                        fs = aligner.process(fs)
                    except Exception as e:
                        log.warning(f"{self._log_name}: align فشل على فريم: {e}")

                cf = fs.get_color_frame()
                if not cf:
                    continue
                color = np.asanyarray(cf.get_data())   # BGR — طلبناها bgr8

                depth = None
                if self.want_depth:
                    df = fs.get_depth_frame()
                    if df:
                        # z16 units -> metres، بالـ scale بتاع الجهاز نفسه
                        depth = (np.asanyarray(df.get_data()).astype(np.float32)
                                 * self._depth_scale)

                self._set_pair(color, depth)
                frames_ok += 1

        except Exception as e:
            log.error(f"{self._log_name}: خطأ غير متوقع: {e}")
        finally:
            try:
                pipeline.stop()
            except Exception:
                pass
            self._clear_frame()
            log.info(
                f"{self._log_name}: الكاميرا اتقفلت. (frames ok={frames_ok})"
            )


# ══════════════════════════════════════════════════════════════════════════════
#  ربط الـ interfaces بالـ parent class
# ══════════════════════════════════════════════════════════════════════════════

CameraHub.OpenCV    = _OpenCV    # type: ignore[attr-defined]
CameraHub.UseePlus  = _UseePlus  # type: ignore[attr-defined]
CameraHub.RealSense = _RealSense # type: ignore[attr-defined]


# ══════════════════════════════════════════════════════════════════════════════
#  تشغيل مباشر للاختبار
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import sys
    import cv2
    import logging

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )

    cam_type  = sys.argv[1] if len(sys.argv) > 1 else "opencv"  # opencv / useeplus / realsense
    cam_index = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    flags     = {a.lower() for a in sys.argv[3:]}
    # فلاجز تشخيص للـ useeplus (الافتراضي = التهيئة الآمنة، شغالة على كل الأجهزة):
    #   fullinit → يفعّل EP2 + clear_halt مع بعض (تهيئة الأبلكيشن الكاملة)
    #   ep2      → يفعّل أمر EP2 بس
    #   halt     → يفعّل clear_halt بس
    #   raw      → من غير upscale (640×480 خام)
    _ep2   = bool({"ep2", "fullinit"} & flags)
    _halt  = bool({"halt", "fullinit"} & flags)
    _upsc  = "raw" not in flags

    if cam_type == "opencv":
        cam = CameraHub.OpenCV(camera_index=cam_index)
    elif cam_type == "realsense":
        # فلاجز الـ realsense:  nodepth → colour بس   |   fast → 848x480@15
        _w, _h, _f = (848, 480, 15) if "fast" in flags else (1280, 720, 6)
        cam = CameraHub.RealSense(camera_index=cam_index,
                                  frame_width=_w, frame_height=_h, fps=_f,
                                  depth="nodepth" not in flags)
    else:
        cam = CameraHub.UseePlus(camera_index=cam_index, upscale=_upsc,
                                 ep2_init=_ep2, clear_halt_init=_halt)

    cam.start()
    print(f"[{cam_type}] اضغط Ctrl+C للإيقاف...")

    try:
        # مهلة أطول في وضع الاختبار — عشان نــدي فرصة لإعادة إرسال CONNECT
        if cam.wait_for_frame(timeout=20.0):
            while True:
                frame = cam.get_frame()
                if frame is not None:
                    # لو الكاميرا فيها depth: اكتب العمق في نص الصورة —
                    # أسرع حاجة تقول لك هل الـ depth سليم ولا كله أصفار
                    if hasattr(cam, "depth_at"):
                        h, w = frame.shape[:2]
                        d = cam.depth_at(w // 2, h // 2)
                        txt = f"centre: {d * 1000:.0f} mm" if d else "centre: no depth"
                        cv2.drawMarker(frame, (w // 2, h // 2), (0, 255, 0),
                                       cv2.MARKER_CROSS, 24, 2)
                        cv2.putText(frame, txt, (12, 32), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.8, (0, 255, 0), 2)
                    cv2.imshow("camera_hub", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        cam.stop()
        cv2.destroyAllWindows()
# EOF
