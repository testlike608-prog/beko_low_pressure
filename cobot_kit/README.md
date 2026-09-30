# cobot_kit

نفس فكرة `3D-point-following-cobot` بالظبط — **نقط، والروبوت بيروح لكل نقطة** —
بس مقسومة موديولز تستخدمها من `app.py`. الفرق الوحيد: اللي بيختار النقط هو الـ AI
بدل ما إنت تدوس عليها.

الفولدر القديم و `app.py` محدش لمسهم. الكيت ده **مستقل**: مش بيستورد حاجة من
الفولدر القديم، والكود اللي اتجرب على الـ FR5 الحقيقي منقول زي ما هو.

---

## الفكرة في صورة واحدة

```
 capture positions ──► صور ──► AI ──► pixels (u,v) ──► Locator ──► x,y,z (base) ──► _robot_cycle
   cycle.capture()             cycle.detect()          cycle.locate()                .run()
        Robot + Camera            AiModel               handeye.json               Robot
```

القاعدة اللي ماشي عليها كل حاجة:

* الـ AI بيرجّع **pixels بس** — مش مليمترات، ومش عارف إن فيه روبوت.
* `Locator` هو الوحيد اللي بيحوّل pixel → x,y,z في الـ base (عمق الـ RealSense + `handeye.json`).
* `_robot_cycle` في `app.py` مش فارق معاه مين اختار النقط — إنت بالماوس ولا الـ AI.

عشان كده تغيير الموديل مش بيلمس الروبوت، وتغيير الكاميرا مش بيلمس الموديل.

---

## الملفات

| الملف | شغله |
|---|---|
| `settings.py` | **قيم الخلية دي**: IP الروبوت، الـ tool، الـ home، الكاميرا، الموديل، مواضع التصوير، المسافات |
| `robot.py` | `Robot` — `start()` / `stop()` / `move_to()` / `move_joints()` / `read_input()` / `halt()` |
| `camera.py` | `Camera` — `start()` / `stop()` / `read()` / `shoot(robot)` → `Shot` (صورة + عمق + flange pose + بتتحفظ في `capture/`) |
| `ai.py` | اللي بيختار النقط: `AiModel` (YOLO) — `ClickPicker` (القديم بالماوس) — `FunctionPicker` (أي موديل عندك) — `FixedPoints` (للتجربة) |
| `locate.py` | `Locator` — pixels → x,y,z في الـ base frame |
| `cycle.py` | `capture` / `detect` / `locate` + `point_pose` / `approach_pose` (حسابات الـ pose لـ `_robot_cycle`) |
| `example_app.py` | **الـ App بتاعك بنفس الكومنتات بالظبط** ومتملي بالكيت — انسخ منه |
| `selftest.py` | السيكوانس كلها من غير أي hardware |
| `camera_hub.py` | درايفرات الكاميرا (منقول زي ما هو) |
| `_fairino.py` `_simulator.py` | درايفرات الروبوت (منقولة زي ما هي بكل الـ fixes) |
| `handeye.json` | الـ calibration (نسخة من القديم) |

---

## الاستخدام من app.py

من غير ما تلمس ملفاتك الحالية — ده الشكل اللي هيبقى عليه:

```python
from cobot_kit import Robot, Camera, AiModel, Locator, cycle

class App():
    def __init__(self, scanner_ip):
        self.scannr_ip = scanner_ip
        self.robot_is_connected = False
        self.robot   = Robot()                 # القيم من cobot_kit/settings.py
        self.camera  = Camera()
        self.ai      = AiModel("weld.pt")
        self.locator = Locator()

    def _start(self):
        self.camera.start()                    #connect to camera
        self.robot.start()                     #connect to cobot
        self.ai.start()
        self.robot_is_connected = True
        ...

    def _stop(self):
        self.robot_is_connected = False        # _robot_cycle بيقف عند النقطة الجاية
        self.robot.halt()                      # ويوقف الحركة اللي شغالة دلوقتي
        self.ai.stop()
        self.camera.stop()                     #stop camera connection
        self.robot.stop()                      # home + MANUAL + disconnect

    def _running_loop(self):
        while self.robot_is_connected:
            if self.robot.read_input(0) == 1:  #check te inpout trigger
                self._start_sequance()

    def _start_sequance(self):
        shots  = cycle.capture(self.robot, self.camera)   #move to cap positions + cap images + save
        shots  = cycle.detect(shots, self.ai)             #ai -> x,y pixel
        points = cycle.locate(shots, self.locator)        #pixels -> x,y,z for the cobot
        return self._robot_cycle(points)

    def _robot_cycle(self, points_array):
        # موجودة كاملة في app.py -- لكل نقطة:
        #   روح للنقطة -> طلّع output -> استنى input -> ارجع ورا -> اللي بعدها
```

## `_robot_cycle` — مكتوبة جوه `app.py`

خطوات كل نقطة مكتوبة قدامك سطر سطر عشان تحط بينهم اللي إنت عايزه:

```python
pose     = cycle.point_pose(point, orientation, CLEARANCE_MM)   # الـ tip فين
approach = cycle.approach_pose(pose, APPROACH_MM)               # 60 مم ورا على محور الـ tool

robot.move_to(approach, vel=SPEED)                  # 1. روح للنقطة
robot.move_to(pose, vel=SLOW, linear=True)
robot.write_output(OUTPUT_NO, 1)                    # 2. طلّع الـ output
got_it = robot.wait_input(INPUT_NO, 1, timeout_s=WAIT_S,
                          cancel=lambda: not self.robot_is_connected)  # 3. استنى الـ input
robot.write_output(OUTPUT_NO, 0)
robot.move_to(approach, vel=SLOW, linear=True)      # 4. ارجع ورا
```

* أرقام الـ DO / DI والمسافات والسرعات constants في أول الـ function نفسها.
* نقطة مش reachable بتتسجل وبتتعدّى، والـ cycle بيكمل (الـ IK بيتحل **قبل** الحركة).
* لو الـ input مجاش في `WAIT_S`، النقطة بتتسجل `input timeout` والـ cycle بيكمل.
  عايزه يقف بدل كده؟ حط `break` مكان الـ `print`.
* `_stop` (لما تخلي `robot_is_connected = False`) بيوقف الانتظار على طول، ومش هيروح لنقطة جديدة.
* الـ tool بيفضل بنفس الاتجاه اللي كان عليه أول الـ cycle.
* بترجع list فيها `{"point", "xyz", "ok", "note"}` لكل نقطة — تبعتها للـ db مثلاً.

النسخة الكاملة اللي بتشتغل (بالـ thread وحافة الـ trigger) في `example_app.py`.

> الـ trigger في المثال ده DI على كونترولر الروبوت. لو الـ trigger جاي من الـ
> scanner client بتاعك، استبدل `self.robot.read_input(0)` بالقراءة بتاعتك —
> الكيت مش فارق معاه.

---

## كل خطوة لوحدها

```python
robot.start()                          # connect + clear faults + AUTOMATIC + servos
                                       # + يتأكد إن الـ tool على البندانت صح + home
robot.move_joints([0, -20, -90, -70, 90, 0])
robot.move_to([650, -400, -150, 180, 0, 175])     # mm / deg, base frame
robot.pose()        robot.flange()     robot.joints()
robot.halt()                           # يوقف الحركة الحالية بس (زرار stop)
robot.stop()                           # عمره ما بيرمي exception — آمن في finally

shot = camera.shoot(robot, name="top")  # صورة + flange pose لحظة الصورة
                                        # لو الذراع اتحركت >1 مم وقت الصورة بيرفض

pixels = ai.find(shot)                  # [Pixel(u, v, label, confidence), ...]
points = locator.to_robot(shot, pixels) # [Point(x, y, z, frame="base"), ...]

robot.write_output(0, 1)                # DO على الكونترولر
robot.wait_input(0, 1, timeout_s=30)    # يستنى DI → True / False
robot.can_reach(pose)                   # True / False من غير حركة
```

---

## مواضع التصوير

في `settings.py`:

```python
CAPTURE_POSITIONS = [
    {"joints": [0, -20, -90, -70, 90, 0]},          # joints أأمن — مفيش IK
    {"pose":   [734, -426, -100, 180, 0, 175]},     # أو pose في الـ base
]
```

لو القايمة فاضية → صورة واحدة من مكان الذراع الحالي. لو نفس اللحام ظهر في
صورتين، `cycle.locate()` بيسيبه مرة واحدة بس (أقرب من 5 مم = نفس النقطة).

كل صورة بتتحفظ في `capture/` جنب `app.py` (+ `.npy` للعمق + `_ai.png` عليها النقط
اللي الـ AI اختارها) — فتقدر تشوف الموديل اختار إيه في كل cycle.

---

## الموديل

```python
AiModel("weld.pt")                                  # detect — نص الـ box
AiModel("weld.pt", classes=["weld"], min_confidence=0.55)
AiModel("weld-seg.pt", task="segment")              # centroid الـ mask
AiModel("weld-pose.pt", task="pose")                # كل keypoint نقطة
```

عندك موديل مش YOLO؟ أي function بتاخد صورة وترجع نقط:

```python
def my_model(image):                  # numpy BGR
    return [(640, 360), (700, 400, "weld2", 0.91)]

ai = FunctionPicker(my_model)
```

ولسه الموديل مش جاهز؟ `ClickPicker()` — نفس الـ interface، إنت اللي بتدوس زي الأول.

---

## التجربة

من الفولدر اللي فيه `app.py`:

```
python -m cobot_kit.selftest            كل السيكوانس من غير hardware (simulator + fake camera)
python -m cobot_kit.example_app         الـ App بالكومنتات، على الـ fakes
python -m cobot_kit.example_app --real  على الـ FR5 والـ RealSense الحقيقيين
```

من غير hardware في الكود بتاعك:

```python
Robot(kind="simulator")
Camera(backend="fake")                     # مشهد صناعي
Camera(backend="fake", folder="capture")   # يعيد الصور اللي اتحفظت — capture حقيقي بيبقى test
FixedPoints([(640, 360), (740, 380)])
```

---

## التنصيب

```
pip install numpy opencv-python pyrealsense2 ultralytics
```

والـ Fairino SDK **مش على pypi** — انسخ فولدر `fairino` وحطه جنب `app.py`.

---

## حاجات لازم تعرفها

* `handeye.json` هنا **نسخة** من القديم. إنت غيّرت الـ flange، فبعد ما تعمل
  calibration جديدة حط الملف الجديد هنا. وبيطلعلك تحذير إن البورد 9×7 (الاتنين فردي)
  — ده حقيقي، و 9×6 أو ChArUco بيحلّه.
* `tool` و `user` في `settings.py` **لازم** يطابقوا البندانت (6 و 0). `robot.start()`
  بيتشيك على ده قبل أي حركة ويقولك بجملة لو فيه اختلاف.
* `robot.stop()` مش E-stop. الـ E-stop هو الزرار الأحمر.
