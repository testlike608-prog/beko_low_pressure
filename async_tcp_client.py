"""
نسخة async/await من TCPClient — نفس المنطق ونفس الحمايات:
  * TID بيتولّد جوه الكلاس + رمي الردود المتأخرة
  * قراءة فريم Modbus كامل بطول الـ MBAP
  * framing للبروتوكول النصي
  * watchdog + keepalive + heartbeat
  * listener بيستحمل القطع

محتاجة Python 3.10+ (asyncio.Lock/Event مش بيترابطوا بـ loop وقت الإنشاء).

الاستخدام من كود async:
    client = AsyncTCPClient("192.168.1.10", 502, protocol="modbus", name="io")
    await client.connect()
    client.start_reconnection_watchdog()
    resp = await client.send_request("000100000006010200000008", is_hex=True)

الاستخدام من كود ثريدات أو Qt: شوف AsyncClientRunner تحت.
"""
import asyncio
import inspect
import socket
import threading
import time

import logstore


class AsyncTCPClient:
    MODBUS_MAX_RESYNC_FRAMES = 8
    HEARTBEAT_MAX_FAILS = 3
    TEXT_QUIET_GAP = 0.03          # ثواني سكوت = نهاية الرسالة لو مفيش delimiter
    DRAIN_GAP = 0.005
    RECEIVE_QUEUE_MAX = 1000

    def __init__(self, ip, port, timeout=None, buffer_size=4096,
                 protocol="auto", modbus_timeout=1.0, connect_timeout=3.0,
                 text_delimiter=None, heartbeat=None, watchdog_interval=3.0,
                 name=""):
        """
        :param timeout: timeout البروتوكول النصي. None = يستنى للأبد
        :param protocol: "modbus" | "text" | "auto"
        :param modbus_timeout: أقصى وقت للطلب كله (إرسال + كل الفريمات) — لازم رقم
        :param text_delimiter: نهاية الرسالة النصية (مثلاً b"\\r\\n"). None = بالسكوت
        :param heartbeat: async def بترجّع None لو فشلت، الـ watchdog بيندهها
        """
        if protocol not in ("auto", "modbus", "text"):
            raise ValueError(f"unknown protocol {protocol!r}")
        if not modbus_timeout or modbus_timeout <= 0:
            raise ValueError("modbus_timeout must be a positive number")
        if isinstance(text_delimiter, str):
            text_delimiter = text_delimiter.encode()

        self.name = name
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.modbus_timeout = modbus_timeout
        self.connect_timeout = connect_timeout
        self.buffer_size = buffer_size
        self.protocol = protocol
        self.text_delimiter = text_delimiter
        self.heartbeat = heartbeat
        self.watchdog_interval = watchdog_interval

        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None
        self.connected = False

        self._stop_event = asyncio.Event()
        # نفس فكرة القفل في نسخة الثريدات: (إرسال + استقبال) عملية واحدة.
        # من غيره اتنين tasks بيستنوا على نفس الـ reader هياخدوا ردود بعض.
        self._io_lock = asyncio.Lock()
        # لو طلب اتلغى (cancel) في نص الفريم، الستريم ممكن يبقى مزحلق —
        # بنعلّم عليه ونفضّيه في أول الطلب الجاي
        self._dirty = False

        self._tid = 0
        self._stale_frames = 0
        self._stale_last_report = 0.0

        self._watchdog_task: asyncio.Task | None = None
        self._listen_task: asyncio.Task | None = None
        self._listening = False
        self.receive_queue: asyncio.Queue = asyncio.Queue(maxsize=self.RECEIVE_QUEUE_MAX)

        self._log_lock = threading.Lock()
        self._log_seq = 0
        self._log = list()
        self._rate_limit = {}

    # ==================================================================
    # Stop / restart
    # ==================================================================
    def is_stopping(self) -> bool:
        return self._stop_event.is_set()

    def reset_stop_flag(self):
        self._stop_event.clear()

    async def stop(self):
        """إيقاف الـ watchdog والـ listener وقفل الاتصال"""
        self._stop_event.set()
        self._listening = False
        tasks = [t for t in (self._watchdog_task, self._listen_task) if t and not t.done()]
        for t in tasks:
            t.cancel()
        # من غير القفل عن قصد: القفل بيصحّي أي طلب واقف مستني رد
        self._close()
        await asyncio.gather(*tasks, return_exceptions=True)

    async def disconnect(self):
        await self.stop()
        self._log_add("INFO", f"[{self.ip}] Connection Closed.")

    # ==================================================================
    # الاتصال
    # ==================================================================
    def _close(self):
        """sync عن قصد: في asyncio مفيش حاجة تقاطعها طالما مفيش await"""
        self.connected = False
        if self._writer is not None:
            try:
                self._writer.close()
            except Exception:
                pass
        self._reader = self._writer = None
        self._dirty = False

    @staticmethod
    def _tune_socket(sock):
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            if hasattr(socket, "SIO_KEEPALIVE_VALS"):             # Windows
                sock.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 3000, 1000))
            else:                                                 # Linux
                for opt, val in (("TCP_KEEPIDLE", 3), ("TCP_KEEPINTVL", 1), ("TCP_KEEPCNT", 3)):
                    if hasattr(socket, opt):
                        sock.setsockopt(socket.IPPROTO_TCP, getattr(socket, opt), val)
        except OSError:
            pass

    async def connect(self):
        """محاولة اتصال واحدة"""
        if self._stop_event.is_set():
            return False

        error = None
        async with self._io_lock:
            if self.connected and self._writer is not None:
                return True
            self._close()
            # بنعمل السوكيت بإيدينا عشان نظبط keepalive (ioctl على Windows)
            # قبل ما asyncio يلفّه
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setblocking(False)
            try:
                loop = asyncio.get_running_loop()
                await asyncio.wait_for(loop.sock_connect(sock, (self.ip, self.port)),
                                       self.connect_timeout)
                self._tune_socket(sock)
                self._reader, self._writer = await asyncio.open_connection(sock=sock)
            except (OSError, asyncio.TimeoutError) as e:
                sock.close()
                error = e or "timeout"
            else:
                self.connected = True

        if error is None:
            self._log_add("INFO", f"[{self.ip}]:[{self.port}] Connected successfully.")
            return True
        self._log_add("WARNING", f"[{self.ip}]:[{self.port}] Connection Failed: {error!r}")
        return False

    async def _sleep_or_stop(self, seconds):
        """True لو اتعمل Stop أثناء الانتظار"""
        try:
            await asyncio.wait_for(self._stop_event.wait(), seconds)
            return True
        except asyncio.TimeoutError:
            return False

    async def ensure_connected(self):
        while not self.connected and not self._stop_event.is_set():
            if await self.connect():
                break
            self._log_add("WARNING", "Retrying in 5 seconds...")
            if await self._sleep_or_stop(5):
                break

    def _watchdog_alive(self):
        return self._watchdog_task is not None and not self._watchdog_task.done()

    def start_reconnection_watchdog(self):
        """لازم تتنده من جوه الـ event loop"""
        if self._watchdog_alive():
            return
        if self._stop_event.is_set():
            self._log_add("WARNING", "Watchdog not started: client is stopped (call reset_stop_flag first)")
            return
        self._watchdog_task = asyncio.get_running_loop().create_task(self._connection_monitor())

    async def _connection_monitor(self):
        heartbeat_fails = 0
        while not self._stop_event.is_set():
            if not self.connected:
                heartbeat_fails = 0
                await self.ensure_connected()

            elif self.heartbeat is not None:
                try:
                    ok = (await self.heartbeat()) is not None
                except asyncio.CancelledError:
                    raise
                except Exception as e:
                    ok = False
                    self._log_add("WARNING", f"Heartbeat error: {e}")
                heartbeat_fails = 0 if ok else heartbeat_fails + 1
                if heartbeat_fails >= self.HEARTBEAT_MAX_FAILS:
                    self._log_add("WARNING", "Heartbeat failed repeatedly - forcing reconnect")
                    async with self._io_lock:
                        self._close()
                    heartbeat_fails = 0

            elif not self._probe():
                self._log_add("WARNING", "Connection lost in background!")
                self._close()

            if await self._sleep_or_stop(self.watchdog_interval):
                break

    def _probe(self):
        """الطرف التاني قفل؟ (السلك المشال بيتكشف بالـ keepalive)"""
        if self._listening or self._io_lock.locked():
            return True
        r, w = self._reader, self._writer
        if r is None or w is None:
            return False
        return not (w.is_closing() or r.at_eof())

    def get_local_address(self):
        return self._writer.get_extra_info("sockname") if self._writer else None

    # ==================================================================
    # مساعدات الاستقبال
    # ==================================================================
    async def _drain_reader(self, reader):
        """تفضية أي ردود متأخرة في البافر. تحت القفل."""
        self._dirty = False
        while True:
            try:
                chunk = await asyncio.wait_for(reader.read(self.buffer_size), self.DRAIN_GAP)
            except (asyncio.TimeoutError, OSError):
                break
            if not chunk:
                break

    @staticmethod
    async def _recv_modbus_frame(reader):
        header = await reader.readexactly(6)
        if header[2:4] != b"\x00\x00":
            raise ValueError(f"bad protocol id {header[2:4].hex()}")
        length = int.from_bytes(header[4:6], "big")
        if not (2 <= length <= 254):
            raise ValueError(f"bad MBAP length {length}")
        return header + await reader.readexactly(length)

    async def _recv_text(self, reader):
        if self.text_delimiter:
            # readuntil بتسيب اللي بعد الـ delimiter في البافر للرسالة الجاية
            return await reader.readuntil(self.text_delimiter)

        buf = await reader.read(self.buffer_size)
        if not buf:
            raise ConnectionResetError("peer closed the connection")
        while True:
            try:
                chunk = await asyncio.wait_for(reader.read(self.buffer_size), self.TEXT_QUIET_GAP)
            except asyncio.TimeoutError:
                break                           # سكت = الرسالة خلصت
            if not chunk:
                break
            buf += chunk
        return buf

    def _take_stale_report(self):
        if not self._stale_frames:
            return 0
        now = time.time()
        if now - self._stale_last_report < 5:
            return 0
        self._stale_last_report = now
        count, self._stale_frames = self._stale_frames, 0
        return count

    # ==================================================================
    # Modbus
    # ==================================================================
    def _next_tid(self):
        self._tid = (self._tid + 1) & 0xFFFF
        return self._tid.to_bytes(2, "big")

    @staticmethod
    def _looks_like_modbus(data, message, is_hex):
        return (
            (is_hex or isinstance(message, bytes))
            and len(data) >= 8
            and data[2:4] == b"\x00\x00"
            and int.from_bytes(data[4:6], "big") == len(data) - 6
        )

    async def _modbus_transaction(self, reader, writer, data, logs):
        tid = self._next_tid()
        frame = tid + data[2:]
        fc = frame[7]

        writer.write(frame)
        await writer.drain()

        # deadline واحد للطلب كله، مش timeout لكل فريم
        loop = asyncio.get_running_loop()
        deadline = loop.time() + self.modbus_timeout
        for _ in range(self.MODBUS_MAX_RESYNC_FRAMES):
            remaining = deadline - loop.time()
            if remaining <= 0:
                raise asyncio.TimeoutError()
            resp = await asyncio.wait_for(self._recv_modbus_frame(reader), remaining)
            if resp[:2] != tid:
                self._stale_frames += 1
                continue
            if resp[7] == (fc | 0x80):
                code = resp[8] if len(resp) > 8 else -1
                if self._should_log(f"mbexc{fc}{code}"):
                    logs.append(("WARNING", f"Modbus exception: fc={fc} code={code}"))
                return None
            if resp[7] != fc:
                if self._should_log("mbfc"):
                    logs.append(("WARNING", f"Modbus fc mismatch: sent {fc}, got {resp[7]}"))
                return None
            return resp

        logs.append(("ERROR", "could not resync modbus stream"))
        await self._drain_reader(reader)
        return None

    # ==================================================================
    # الإرسال والاستقبال
    # ==================================================================
    @staticmethod
    def _encode(message, is_hex):
        if isinstance(message, bytes):
            return message
        if is_hex:
            return bytes.fromhex(message)
        return message.encode("utf-8")

    async def send_request(self, message, is_hex=False, protocol=None):
        """بترجّع الرد، أو None لو حصل أي فشل"""
        if self._stop_event.is_set():
            return None
        if self._listening:
            self._log_add("ERROR", "send_request called while listening - refused (would steal data)")
            return None

        if not self.connected or self._writer is None:
            if self._watchdog_alive() or not await self.connect():
                return None

        try:
            data = self._encode(message, is_hex)
        except ValueError as e:
            self._log_add("ERROR", f"Bad message: {e}")
            return None

        proto = protocol or self.protocol
        is_modbus = proto == "modbus" or (
            proto == "auto" and self._looks_like_modbus(data, message, is_hex))
        if is_modbus and len(data) < 8:
            self._log_add("ERROR", "Modbus message too short")
            return None

        logs = []
        result = None
        async with self._io_lock:
            reader, writer = self._reader, self._writer
            if reader is None or writer is None:
                return None
            try:
                if self._dirty:
                    await self._drain_reader(reader)

                if is_modbus:
                    result = await self._modbus_transaction(reader, writer, data, logs)
                else:
                    writer.write(data)
                    await writer.drain()
                    result = await asyncio.wait_for(self._recv_text(reader), self.timeout)

            except asyncio.TimeoutError:
                logs.append(("WARNING", f"[{self.ip}]:[{self.port}] Timeout: Server took too long to respond."))
                await self._drain_reader(reader)

            except (ValueError, asyncio.LimitOverrunError) as e:
                logs.append(("WARNING", f"[{self.ip}]:[{self.port}] Bad frame ({e}) - draining"))
                await self._drain_reader(reader)

            except (OSError, EOFError) as e:    # IncompleteReadError = EOFError
                logs.append(("WARNING", f"[{self.ip}]:[{self.port}] Connection Lost ({e!r})."))
                if self._reader is reader:
                    self._close()

            except asyncio.CancelledError:
                # حد لغى الطلب في النص — الستريم ممكن يبقى مزحلق
                self._dirty = True
                raise

            except Exception as e:
                logs.append(("ERROR", f"[{self.ip}]:[{self.port}] General Error: {e!r}"))

            stale = self._take_stale_report()

        for level, msg in logs:
            self._log_add(level, msg)
        if stale:
            self._log_add("WARNING", f"{stale} stale modbus frame(s) dropped in the last 5s")
        return result

    # ==================================================================
    # الاستماع (للأجهزة اللي بتبعت لوحدها)
    # ==================================================================
    def start_listening(self, callback=None):
        """callback ممكن تبقى def عادية أو async def. لازم تتنده من جوه الـ loop."""
        if self._listen_task is not None and not self._listen_task.done():
            return
        self._listening = True
        self._listen_task = asyncio.get_running_loop().create_task(self._listen_loop(callback))
        self._log_add("INFO", f"[{self.ip}]:[{self.port}] Started listening for incoming data...")

    def stop_listening(self):
        self._listening = False

    async def _listen_loop(self, callback):
        while self._listening and not self._stop_event.is_set():
            reader = self._reader
            if not self.connected or reader is None:
                if await self._sleep_or_stop(0.5):
                    break
                continue

            try:
                data = await asyncio.wait_for(reader.read(self.buffer_size), self.timeout)
            except asyncio.TimeoutError:
                continue
            except (OSError, EOFError) as e:
                if self.connected and not self._stop_event.is_set():
                    self._log_add("WARNING", f"[{self.ip}] Listening Error: {e!r}")
                    self._close()
                continue

            if not data:
                self._log_add("WARNING", f"[{self.ip}] Server closed the connection.")
                self._close()
                continue

            self._queue_put(data)
            if callback:
                try:
                    res = callback(data)
                    if inspect.isawaitable(res):
                        await res
                except asyncio.CancelledError:
                    raise
                except Exception as e:
                    self._log_add("ERROR", f"Listen callback error: {e!r}")

    def _queue_put(self, data):
        try:
            self.receive_queue.put_nowait(data)
        except asyncio.QueueFull:
            try:
                self.receive_queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
            self.receive_queue.put_nowait(data)

    async def get_last_received(self, block=False, timeout=None):
        try:
            if not block:
                return self.receive_queue.get_nowait()
            return await asyncio.wait_for(self.receive_queue.get(), timeout)
        except (asyncio.QueueEmpty, asyncio.TimeoutError):
            return None

    # ==================================================================
    # لوج
    # ==================================================================
    def _should_log(self, key, interval=5.0):
        now = time.time()
        if now - self._rate_limit.get(key, 0) < interval:
            return False
        self._rate_limit[key] = now
        return True

    def _log_add(self, level: str, msg: str):
        # threading.Lock عشان _log ممكن يتقري من ثريد الـ GUI
        with self._log_lock:
            self._log_seq += 1
            self._log.append((self._log_seq, time.time(), level, msg))
            if len(self._log) > 5000:
                self._log = self._log[-3000:]
        print(f"[{self.name}][{level}] {msg}")
        # تنبيه: كتابة sync على الديسك جوه الـ loop. لو logstore بطيء،
        # خليه يكتب من ثريد لوحده (QueueHandler مثلاً).
        logstore.write(self.name or "unnamed", level, msg)


# ======================================================================
# جسر لكود الثريدات / Qt
# ======================================================================
class AsyncClientRunner:
    """
    بيشغّل event loop في ثريد لوحده، عشان كود sync (ثريدات أو أزرار Qt)
    يستخدم الكلاينتات الـ async من غير ما يتحول هو لـ async.

        runner = AsyncClientRunner()
        io = runner.call(AsyncTCPClient, "192.168.1.10", 502, protocol="modbus", name="io")
        runner.run(io.connect())
        runner.call(io.start_reconnection_watchdog)
        resp = runner.run(io.send_request(req_hex, is_hex=True), timeout=2)
        ...
        runner.run(io.stop()); runner.close()
    """

    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self.loop.run_forever, daemon=True, name="asyncio-io")
        self._thread.start()

    def run(self, coro, timeout=None):
        """تشغيل coroutine على الـ loop واستنى النتيجة من أي ثريد"""
        return asyncio.run_coroutine_threadsafe(coro, self.loop).result(timeout)

    def call(self, fn, *args, **kwargs):
        """تشغيل فانكشن عادية جوه ثريد الـ loop (مثلاً إنشاء الكلاينت أو start_*)"""
        async def _wrapper():
            return fn(*args, **kwargs)
        return self.run(_wrapper())

    def close(self):
        self.loop.call_soon_threadsafe(self.loop.stop)
        self._thread.join(timeout=2)
