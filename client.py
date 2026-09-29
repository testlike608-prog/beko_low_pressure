import socket
import threading
import time
import queue
import logstore
















# General Class
class  TCPClient():
    def __init__(self, ip, port, timeout=None, buffer_size=4096):
        """
        :param timeout: لو خليته None هيفضل مستني للأبد لحد ما السيرفر يرد
        """
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.buffer_size = buffer_size
        self.sock = None  # هنا هنحتفظ بالسوكيت عشان يفضل مفتوح
        self.connected = False
        # يتفعّل عند الضغط على Stop لإيقاف كل اللوبات الخلفية بشكل نظيف
        self._stop_event = threading.Event()
        # ------------------------------------------------------------------
        # قفل السوكيت.
        #
        # أكتر من ثريد بيستخدموا نفس الـ TCPClient (لوب القراءة + سيكونس
        # محطة 1 + سيكونس محطة 2). من غير القفل ده، ثريد ممكن يعمل recv
        # فياخد رد الطلب بتاع ثريد تاني — والثريد التاني يستنى لحد الـ
        # timeout. اللي كان بيحصل: قراءة DI0 تاخد قيمة DI1 أو DI2، والحالة
        # السابقة تتغير من غير ما الحساس يتحرك، فتتخلق حافة صاعدة وهمية
        # والسيكونس يتنده تاني والإضاءة تنور وتطفي طول ما الحساس قارئ.
        #
        # القفل بيخلي (إرسال + استقبال) عملية واحدة مش قابلة للتقسيم.
        # أمر Modbus واحد بياخد ~5ms، فمفيش أي تعطيل محسوس بين المحطتين.
        # ------------------------------------------------------------------
        self._io_lock = threading.RLock()
        # عدّاد الفريمات المتأخرة اللي اترمت — بيتطبع ملخّص كل 5 ثواني
        # بدل سطر لكل واحدة، عشان الترمينال ما يغرقش
        self._stale_frames = 0
        self._stale_last_report = 0.0
        self._send_queue: "queue.Queue[dict]" = queue.Queue()
        self._log_lock = threading.Lock()
        self._log_seq = 0
        self._log = list()
        self.name =""
        self.current_program_label =""
        self.current_program_data=""

        self.shared_queue = queue.Queue()
        self.shared_queue2= queue.Queue() #FOR DUMMY shared between scanner and data proccesing function 
        self.shared_queue3= queue.Queue() # for dummies shared between scanner and i/o writer function

    # ------------------------------------------------------------------
    # Stop / restart support (used by the Start & Stop buttons in the UI)
    # ------------------------------------------------------------------
    def is_stopping(self) -> bool:
        return self._stop_event.is_set()

    def reset_stop_flag(self):
        """يُستدعى قبل Start عشان اللوبات تشتغل من جديد"""
        self._stop_event.clear()

    def stop(self):
        """إيقاف كل اللوبات الخلفية وقفل السوكيت"""
        self._stop_event.set()
        self.connected = False
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None

    def connect(self):
        """دالة لفتح الاتصال مرة واحدة"""
        if self._stop_event.is_set():
            return False
        try:
            if self.connected:
                    return True
            
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout) # تحديد وقت الانتظار (أو None للانتظار الدائم)
            self.sock.connect((self.ip, self.port))
            self.connected = True
            print(f"[{self.ip}] : [{self.port}] Connected successfully.")
            return True
        except Exception as e:
            print(f"[{self.ip}] : [{self.port}] Connection Failed: {e}")
            self.connected = False
            return False
            
    def ensure_connected(self):
        """تتأكد إننا متصلين، ولو مش متصلين تحاول للأبد (إلا لو اتعمل Stop)"""
        while not self.connected and not self._stop_event.is_set():
            self._log_add("INFO", f"Trying to reconnect to {self.ip}...")
            if self.connect():
                self._log_add("INFO", "Reconnected successfully!")
                break
            else:
                self._log_add("WARNING", "Retrying in 5 seconds...")
                if self._stop_event.wait(5):
                    break

    def start_reconnection_watchdog(self):
        """تشغيل خيط المراقبة في الخلفية"""
        self._stop_event.clear()
        thread = threading.Thread(target=self._connection_monitor, daemon=True)
        thread.start()

    def _connection_monitor(self):
        """الدالة اللي بتراقب الاتصال كل كام ثانية"""
        while not self._stop_event.is_set():
            if not self.connected:
                # لو لقيناه فصل، نصلحه
                self.ensure_connected()
            else:
                # لو متصل، نتأكد إنه "فعلاً" لسه شغال.
                #
                # قبل كده كانت بتبعت self.sock.send(b'', socket.MSG_OOB) —
                # كتابة على نفس السوكيت من ثريد تالت وخارج أي قفل، وكل 3
                # ثواني، فكانت بتقدر تزحلق ستريم الردود. دلوقتي بنفحص
                # السوكيت من غير ما نكتب عليه حاجة خالص.
                try:
                    with self._io_lock:
                        if self.sock is None:
                            raise OSError("socket is gone")
                        self.sock.fileno()          # بيرمي OSError لو اتقفل
                except Exception:
                    self._log_add("WARNING", "Connection lost in background!")
                    self.connected = False

            if self._stop_event.wait(3):  # افحص كل 3 ثواني
                break
    
    def _get_sock(self):
         
        local_ip, local_port = self.sock.getsockname()
        return local_ip,local_port
   
    # ------------------------------------------------------------------
    # مساعدات الاستقبال
    # ------------------------------------------------------------------
    def _drain_socket(self):
        """
        بتفضّي أي ردود متأخرة لسه في البافر.

        مهمة بعد أي timeout: الرد المتأخر بيفضل مستني في السوكيت، ولو
        سبناه هيتسرق من الطلب اللي بعده وكل الردود بعد كده تبقى مزحلقة
        بواحد — وده بيخلي قراءة DI0 ترجّع قيمة DI1 للأبد.
        """
        if self.sock is None:
            return
        try:
            self.sock.setblocking(False)
            while True:
                if not self.sock.recv(self.buffer_size):
                    break
        except (BlockingIOError, OSError):
            pass
        finally:
            try:
                self.sock.settimeout(self.timeout)
            except OSError:
                pass

    def _report_stale_frames(self):
        """ملخّص الفريمات المتأخرة، مرة كل 5 ثواني على الأكثر."""
        if not self._stale_frames:
            return
        now = time.time()
        if now - self._stale_last_report < 5:
            return
        self._stale_last_report = now
        count, self._stale_frames = self._stale_frames, 0
        self._log_add("WARNING", f"{count} stale modbus frame(s) dropped in the last 5s")

    def _recv_exactly(self, count):
        """بتقرا عدد بايتات محدد بالظبط، أو بترمي socket.timeout."""
        buf = b""
        while len(buf) < count:
            chunk = self.sock.recv(count - len(buf))
            if not chunk:
                raise ConnectionResetError("peer closed the connection")
            buf += chunk
        return buf

    def _recv_modbus_frame(self):
        """
        بتقرا فريم Modbus/TCP كامل بالظبط — مش أول حاجة تيجي من البافر.

        الفريم = MBAP(6) + الطول المكتوب في البايتات 4:6.
        القراءة بالطول دي هي اللي بتمنع إن فريمين يتلزقوا في recv واحدة أو
        إن نص فريم يتقرا ويتحسب رد كامل.
        """
        header = self._recv_exactly(6)
        length = int.from_bytes(header[4:6], "big")
        if not (1 <= length <= 253):
            raise ValueError(f"bad MBAP length {length}")
        return header + self._recv_exactly(length)

    def send_request(self, message , is_hex=False):
        """
        إرسال واستقبال فقط (بدون إغلاق الاتصال)

        كل الكلام ده بيحصل جوه self._io_lock عشان يفضل (إرسال + استقبال)
        عملية واحدة. من غير القفل ده، ثريد ممكن ياخد رد ثريد تاني.
        """
        # بعد الضغط على Stop مش بنحاول نبعت أو نعيد الاتصال
        if self._stop_event.is_set():
            return None

        if not self.connected or self.sock is None:
            print(f"[{self.ip}]:[{self.port}] Error: Not connected! Trying to connect...")
            self.ensure_connected()
            if not self.connected or self.sock is None:
                return None


        try:
            # 1. تجهيز الرسالة
            data_to_send = None
            if isinstance(message, bytes):
                data_to_send = message
            elif is_hex:
                data_to_send = bytes.fromhex(message)
            else:
                data_to_send = message.encode('utf-8')
                #data_to_send = [chunk.encode('utf-8') for chunk in message]

            # أوامر Modbus بس هي اللي ليها MBAP وTransaction ID.
            # باقي الأجهزة (السكانر، الفيجن، كاميرا الكابتشر) بروتوكول نصي.
            is_modbus = (
                (is_hex or isinstance(message, bytes))
                and len(data_to_send) >= 8
                and data_to_send[2:4] == b"\x00\x00"
            )
            expected_tid = data_to_send[:2] if is_modbus else None

            with self._io_lock:
                # 2. الإرسال
                self.sock.sendall(data_to_send)

                # 3. الاستقبال
                if not is_modbus:
                    return self.sock.recv(self.buffer_size)

                # Modbus: نقرا فريمات كاملة لحد ما نلاقي الرد بتاع الطلب ده.
                # أي فريم برقم قديم هو رد متأخر من طلب سابق — نرميه ونكمّل،
                # وبكده السوكيت بيرجع متزامن لوحده بدل ما يفضل مزحلق.
                for _ in range(8):
                    response = self._recv_modbus_frame()
                    if response[:2] == expected_tid:
                        return response
                    # لوب القراءة بتلف 20 مرة في الثانية، فلو الموديول بقى
                    # تعبان الرسالة دي ممكن تغرق الترمينال. بنعدّها ونطبعها
                    # مرة كل 5 ثواني بالعدد — المعلومة بتوصل من غير سيل لوج.
                    self._stale_frames += 1

                self._log_add("ERROR", "could not resync modbus stream")
                self._drain_socket()
                return None

        except (socket.timeout):
            self._log_add("WARNING", f"[{self.ip}]:[{self.port}] Timeout: Server took too long to respond.")
            # الرد المتأخر لازم يتشال من البافر، وإلا هيتسرق من الطلب الجاي
            with self._io_lock:
                self._drain_socket()
            return None

        except ValueError as e:
            # فريم مش مفهوم (طول غلط) — نفضّي ونكمّل بدل ما نبني على داتا غلط
            self._log_add("WARNING", f"[{self.ip}]:[{self.port}] Bad frame ({e}) - draining")
            with self._io_lock:
                self._drain_socket()
            return None

        except (OSError, BrokenPipeError, ConnectionResetError, socket.error) as e:
            # هنا أهم تعديل: لو حصل أي خطأ في السوكيت (السيرفر قفل أو السلك اتشال)
            print(f"[{self.ip}]:[{self.port}] Connection Lost ({e}). Reconnecting...")
            
            self.connected = False
            if self.sock:
                try:
                    self.sock.close()
                except:
                    pass
                self.sock = None
            
            # محاولة إعادة الاتصال فوراً
            self.ensure_connected()
            
            # اختياري: ممكن تخليها تحاول تبعت الرسالة تاني بعد ما رجع الاتصال
            # return self.send_request(message, is_hex) 
            return None

        except Exception as e:
            print(f"[{self.ip}]:[{self.port}] General Error: {e}")
            return None

        finally:
            # ملخّص الفريمات المتأخرة (لو في) - مرة كل 5 ثواني بالكتير
            self._report_stale_frames()

    '''
    def _start_monitoring(self):
        """بدء خيط المراقبة"""
        if self._monitor_thread is None or not self._monitor_thread.is_alive():
            self._stop_monitor.clear()
            self._monitor_thread = threading.Thread(target=self._monitor_connections, daemon=True)
            self._monitor_thread.start()

    def _monitor_connections(self):
        """فانكشن المراقبة اللي بتشيك على حالة الاتصال كل فترة"""
        print(f"[{self.ip}] Connection monitor started.")
        while not self._stop_monitor.is_set():
            if self.connected and self.sock:
                try:
                    # بنبعث "بيانات فارغة" عشان نختبر لو السوكيت لسه شغال (Keep-alive check)
                    # MSG_PEEK بيشوف البيانات من غير ما يسحبها من البافر
                    self.sock.send(b"", socket.MSG_DONTWAIT)
                except (OSError, BrokenPipeError):
                    print(f"[{self.ip}] Monitor detected broken connection!")
                    self.connected = False
                    # هنا ممكن تختار تنادي self.connect() تاني لو عايز Auto-reconnect
                    break
            time.sleep(5)  # شيك كل 5 ثواني مثلاً
     
   '''
    
    def disconnect(self):
        """إغلاق الاتصال وإيقاف المونيتور"""
        self._stop_event.set()  # وقف اللوب في المونيتور
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        self.sock = None
        self.connected = False
        print(f"[{self.ip}] Connection Closed.")

    def _log_add(self, level: str, msg: str):
        with self._log_lock:
            self._log_seq += 1
            self._log.append((self._log_seq, time.time(), level, msg))
            if len(self._log) > 5000:
                self._log = self._log[-3000:]
        print(f"[{self.name}][{level}] {msg}")
        # نسخة دائمة على الديسك: اللي في الرامة بيتقص وبيضيع مع القفل
        logstore.write(self.name or "unnamed", level, msg)
    
    def start_listening(self, callback=None):
        """
        دالة لبدء عملية الاستماع في Thread منفصل
        :param callback: دالة اختيارية يتم استدعاؤها فور استلام بيانات
        """
        self.receive_queue = queue.Queue() # كيو لاستقبال البيانات
        self._stop_event.clear()
        self.listen_thread = threading.Thread(target=self._listen_loop, args=(callback,), daemon=True)
        self.listen_thread.start()
        self._log_add("INFO", f"[{self.ip}] : [{self.port}] Started listening for incoming data...")
        

    def _listen_loop(self, callback):
        """الـ Loop الداخلي اللي بيفضل مستني داتا"""
        while self.connected and not self._stop_event.is_set():
            try:
                # الكود هيفضل واقف هنا لحد ما السيرفر يبعت حاجة
                data = self.sock.recv(self.buffer_size)
                
                if not data:
                    # لو السيرفر بعت داتا فاضية معناها قفل الاتصال
                    print(f"[{self.ip}] Server closed the connection.")
                    self.connected = False
                    break
                
                if callback:
                    callback(data)
                # إضافة البيانات للكيو
                #self.receive_queue.put(data)

                # اختياري: تسجيل اللوج
                # self._log_add("INFO", f"Received data: {data}")

            except socket.timeout:
                continue # لو حصل تايم أوت يرجع يحاول يستقبل تاني
            except Exception as e:
                if self.connected:
                    print(f"[{self.ip}] Listening Error: {e}")
                    self.connected = False
                break

    def get_last_received(self, block=False, timeout=None):
        """دالة لسحب آخر داتا وصلت من الكيو"""
        try:
            return self.receive_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return None
