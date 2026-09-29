# weld-leak-inspector

كاميرا بتشوف نقط اللحام بالـ AI، والكوبوت بياخد الـ sniffer ويروح على كل نقطة
يختبرها leak.

المشروع مقسوم لأربع حتت مستقلة تماماً. كل حتة ليها نفس الشكل: `start()` و
`stop()` و ملف driver لكل موديل.

```
camera/    الكاميرا      realsense | webcam | fake
robot/     الكوبوت       fairino   | simulator
vision/    الموديل       yolo | clicks | fixed | file
sniffer/   جهاز الـ leak fake | digital

core/      الرياضيات المشتركة — مفيهاش أي hardware خالص
cell.py    الـ cycle: capture → detect → hand-eye → visit → sniff → report
main.py    التطبيق
check.py   سلم الاختبار (الأهم — إقرا القسم بتاعه تحت)
settings.py باقي الإعدادات (الكاميرا نفسها في main.py)
```

القاعدة اللي ماشي عليها كل حاجة:

> الكاميرا شغلها بيخلص عند: **pixel + depth → نقطة في الـ camera frame**.
> الكوبوت شغله بيبدأ من: **نقطة في الـ base frame**.
> اللي بينهم حاجة واحدة بس: الـ hand-eye transform في `core/handeye.py`.

عشان كده تغيير الكاميرا مش بيلمس الكوبوت، وتغيير الكوبوت مش بيلمس الكاميرا،
والموديل لما يتدرب بيدخل في ملف واحد بس.

---

## الـ start و الـ stop

ده اللي طلبته بالظبط — كل حاجة وراها دالتين:

```python
import robot, camera, vision, sniffer

robot.start()      # connect + clear faults + AUTOMATIC + servos on
                   # + verify frames + set speed + home
camera.start()     # يفتح الكاميرا اللي مختارة في settings.py
vision.start()     # يحمّل الموديل
sniffer.start()

# ... شغل ...

robot.stop()       # يوقف الحركة + (home) + MANUAL + disconnect
camera.stop()
vision.stop()
sniffer.stop()
```

ثلاث ضمانات مهمة في `robot.stop()`:

1. **عمره ما بيرمي exception.** بيتنادى جوه `finally` وفي الـ exception handler
   وعلى Ctrl+C — أحياناً الاتنين في نفس اللحظة. كل خطوة بتتجرب لوحدها: كونترولر
   مش راضي يعمل home لازم برضه يرجع MANUAL، واللي مش راضي يغيّر الـ mode لازم
   برضه يتقفل.
2. **آمن تنادي عليه مرتين**، وآمن تنادي عليه من غير ما تعمل `start()` أصلاً.
3. **`robot.start()` لو فشل في النص، بيرجّع الذراع تاني.** مش هتلاقي ذراع
   نص-شغال سايب في AUTOMATIC والـ servos on.

وفيه كمان:

```python
robot.halt()       # يوقف الحركة الحالية بس ويفضل connected — ده زرار الـ stop
                   # بتاع الـ run، مش shutdown
```

`halt()` بيفتح connection تاني لوحده، لأن الأول مقفول جوه الحركة اللي بيقاطعها.

---

## سلم الاختبار — `check.py`

ده أهم ملف في المشروع عندك. كل درجة بتضيف **حاجة واحدة بس** ممكن تكون غلط:

```
python check.py 1     imports + settings + رياضيات الـ hand-eye    مفيش hardware
python check.py 2     يفتح الكاميرا، ياخد frame، يحفظها
python check.py 3     يتصل بالكوبوت ويقراه          — مفيش أي حركة
python check.py 4     jog صغير بطيء                 [الذراع بتتحرك]
python check.py 5     تدوس على نقطة لحام وتشوف الكوبوت فاهمها فين
python check.py 6     cycle كامل، planning بس، مفيش حركة
python check.py 7     cycle كامل حقيقي              [الذراع بتتحرك]
```

الفايدة الحقيقية: لما درجة 5 تطلّع رقم غلط بـ 40 مم، درجات 1–4 تكون أثبتت إن
المشكلة **مش** في الرياضيات، ولا الكاميرا، ولا الاتصال، ولا الحركة — يبقى هي في
الـ calibration أو في الـ tool اللي على الـ pendant، ومفيش مكان تاني تدور فيه.

درجة 4 و 7 بيقولوا لك إن الذراع هتتحرك وبيستنوا منك `YES`.

وكمان:

```
python -m pytest tests -q      89 test، أقل من ثانية، من غير أي hardware
```

---

## التشغيل من غير أي hardware

في `settings.py` غيّر كلمة واحدة في كل block:

```python
CAMERA = {"kind": "fake", ...}        # مشهد صناعي، أو folder فيه صور محفوظة
ROBOT  = {"kind": "simulator", ...}   # ذراع من حسابات، بيرفض اللي مش في المدى
VISION = {"kind": "fixed", ...}       # نقط ثابتة
SNIFFER = {"kind": "fake"}
```

مش محتاج تمسح باقي المفاتيح — الـ `simulator` بيقبل `ip` و `tool` و `user`
ويتجاهلهم، عشان التبديل يبقى كلمة واحدة. لكن مفتاح مش موجود أصلاً (غلطة طباعة)
لسه بيطلّع error.

بعدها:

```
python main.py --dry      يخطط كل نقطة من غير ما يتحرك حاجة
python main.py            cycle كامل على الـ fakes
```

### تسجيل مشهد حقيقي واستخدامه بعدين

```
python check.py 2                                  بيحفظ captures/check.png + .npy
CAMERA = {"kind": "fake", "folder": "captures"}    وبعدها بيعيدها
```

كده الـ capture الحقيقي بتاع النهاردة بيبقى regression test بكرة.

---

## الـ cycle

```
capture   يروح لـ CAPTURE_POSE (لو متحددة)، ياخد frame، يقرا الـ flange pose
          قبل الصورة وبعدها — لو الذراع اتحركت أكتر من 1 مم بيرفض الـ frame
detect    الموديل بيرجّع pixels — مش مليمترات، ومش عارف إن فيه كوبوت أصلاً
hand-eye  camera mm → base mm، باستخدام الـ flange pose بتاع لحظة الصورة نفسها
plan      لكل نقطة: approach + pose + هل هي reachable — كله قبل أي حركة
visit     approach → النقطة → dwell → sniffer → retreat
report    سطر لكل نقطة، وسطر JSON لكل cycle في results.jsonl
```

نقطة واحدة مش reachable بتتسجّل وبتتعدّى؛ الـ cycle بيكمّل. لكن عدد نقط غلط
(أقل من اللي الـ recipe بيقوله) بيوقف الـ cycle — لأن اللحام اللي الموديل مشافوش
هو بالظبط اللي المفروض يتختبر.

---

## لما الموديل يخلص training

ملف واحد بيتغير — `settings.py`:

```python
VISION = {"kind": "yolo", "weights": "weld.pt", "min_confidence": 0.5}
```

والـ service port (اللي عليه الجوان الأسود واللي الـ sniffer لازم يدخل جواه):
ده محتاج **اتجاه** مش مكان بس. درّب pose model يحدد فتحة البؤبؤ والنهاية
البعيدة، وبعدها:

```python
VISION = {"kind": "yolo", "weights": "weld-pose.pt", "task": "pose",
          "axis_from_keypoints": (0, 1)}
```

ساعتها `cell.plan()` بيلف الـ tool عشان يدخل **على محور** الفتحة بدل ما يخبط
على السطح. ولو واحدة من الـ keypoints مفيش عندها depth، النقطة بتفضل من غير
normal بدل ما يتخرّع اتجاه — اتجاه متخيّل معناه sniffer بيتلوي جوه الجوان.

---

## إضافة كاميرا أو كوبوت جديد

نفس الثلاث خطوات في الحالتين، وملف واحد جديد:

**كاميرا موجودة في `camera_hub.py`** — مفيش ملف جديد أصلاً، كلمة واحدة في
**`main.py`** (هو اللي بيختار الكاميرا، مش `settings.py`):

```python
CAMERA = {"backend": "realsense", "camera_index": 0,
          "frame_width": 1280, "frame_height": 720, "fps": 6}
```

ومن غير هاردوير: `CAMERA = {"backend": "fake"}`. لو سيبت مفاتيح الـ camera_hub
وراك (frame_width وخلافه) مفيش مشكلة — بيتجاهلوا وبيقول لك أنهي واحد،
زي ما الـ simulator بيعمل مع ip و tool.

`check.py` بيستورد `main.build_camera()`، فسلم الاختبار بيفتح **نفس** الكاميرا
اللي الـ run الحقيقي بيفتحها — مفيش مصدرين للحقيقة.

`camera_hub.py` موديول مستقل بيتنسخ زي ما هو جنب `main.py`؛ `camera/hub.py` هو
المكان الوحيد اللي عارف الشكلين. ولما تضيف driver جديد هناك، سطر في
`Hub.BACKENDS` يخليه يشتغل هنا.

فرق واحد مهم بين الاتنين: `camera_hub` **push** (ثريد بيصوّر على طول)،
والمشروع ده **pull** (الذراع تروح الأول وبعدين يطلب صورة). عشان كده
`camera/hub.py` مبياخدش الفريم اللي لاقيه — بيستنى فريم **أحدث من** اللي قبله
(`frame_seq`)، لأن صورة اتاخدت والذراع لسه بتتحرك بتعدي من تحت الـ pose guard
بتاع `cell.py` بسهولة — الذراع واقفة وقت القراءتين، فالاتنين بيتفقوا.

**كاميرا جديدة خالص** — إنسخ `camera/webcam.py` (ده أوضح مثال) لـ `camera/hikrobot.py`،
إملا `open()` / `close()` / `read()` و `lens`، وضيف سطر في `REGISTRY` جوه
`camera/__init__.py`.

**كوبوت** — إنسخ `robot/simulator.py` لـ `robot/abb.py`، إملا السبع دوال، واكتب
الـ capability flags بصدق:

```python
has_ik = False            # مفيش IK؟ الـ cycle لسه هيشتغل — هيبعت poses
can_stop_mid_move = False # مش هتقدر تقاطع حركة؟ الكود هيقول كده بصوت عالي
can_move_linear = False
has_io = True
```

الـ flags دي هي اللي بتخلّي "بيشتغل مع أي كوبوت" جملة صحيحة بدل "بيشتغل مع أي
كوبوت شبه الـ FR5".

مفيش أي حاجة تانية بتتغير — لا `main.py` ولا `cell.py` ولا الموديل.

---

## الحاجات اللي اتكلّفت يوم كل واحدة فيهم على الـ hardware

مكتوبة في `robot/fairino.py` في أول الملف، ومتغطّية بـ tests:

1. **`MoveJ` و `MoveL` بياخدوا نفس الليستتين بترتيب معكوس.** كل argument هنا
   بيتبعت **بالاسم**، فالترتيب بقى مش مهم.
2. **الـ pair لازم يتفقوا** — الـ joints والـ pose المبعوتين مع بعض لو بيوصفوا
   نقطتين مختلفتين، الكونترولر بيرفض كأنها نقطة وحشة مش كأنها تعارض.
3. **`GetInverseKin` مبيخدش tool ولا user** — بيحل في اللي الـ pendant مفعّله،
   بينما `MoveJ`/`MoveL` بياخدوهم صراحة. لما يختلفوا بييجي error 74 أو 154 —
   أرقام عن الـ joints مش عن الـ frames. عشان كده `verify()` بيتشيك على ده
   **قبل** أي حركة، وده أنفع check في الـ driver كله.
4. **الـ stop محتاج connection لوحده** — الأصلي مقفول جوه الحركة.
5. **`robot_state_pkg` بيرجع بثلاث أشكال** حسب الـ build: instance، أو POINTER،
   أو الـ **class** نفسه. في الحالة التالتة كل attribute بيبقى ctypes field
   descriptor و `int()` عليه بيرمي `not '_ctypes.CField'` — رسالة مالهاش أي علاقة
   بالروبوت وبتقتل الـ run قبل أول حركة. `_field()` بيتعامل مع التلاتة، والأهم
   إنه بيرجّع **هل اتقرا فعلاً ولا لأ**: "مش شايف الـ mode" و "الـ mode تمام"
   ردين مختلفين، واللي بيخلط بينهم بيقول لك robot ready وبعدها يرفض كل حركة.

وحاجة سادسة في `core/handeye.py`: الـ `handeye.json` الحالي معمول ببورد
**9×7 inner corners — الاتنين فردي**، يعني OpenCV ممكن يختار أي طرف كأصل،
والـ 20 capture بينقسموا مجموعتين. `HandEye.warnings()` بيقول لك ده كل مرة.
الحل: بورد فيه ضلع زوجي (9×6) أو ChArUco، وإعادة calibration.

---

## الملفات في السطر الواحد

| الملف | شغله |
|---|---|
| `settings.py` | كل باقي الإعدادات: الكوبوت، الموديل، الـ sniffer، المسافات |
| `camera_hub.py` | درايفرات الكاميرات — موديول مستقل بيتنسخ زي ما هو |
| `camera/hub.py` | المترجم بين camera_hub والمشروع (وحارس الفريم البايت) |
| `main.py` | التطبيق **واختيار الكاميرا** — `--dry` `--loop` `--expect N` `--describe` |
| `check.py` | سلم الاختبار — 7 درجات |
| `cell.py` | ترتيب الـ cycle — الملف الوحيد اللي عارف الترتيب |
| `core/types.py` | `Pose` `Pixel` `Point` `Batch` `Result` + الـ frames |
| `core/geometry.py` | `Rz @ Ry @ Rx` — الـ convention مكتوب مرة واحدة |
| `core/handeye.py` | camera mm → base mm |
| `camera/base.py` | `Lens` `Frame` + `depth_at` / `to_point` |
| `robot/base.py` | الـ interface والـ capability flags |
| `vision/base.py` | `Detector` + `sort_reading_order` + `drop_duplicates` |
| `handeye.json` | الـ calibration — متتغيرش إلا لما الـ bracket يتحرك |
| `points.csv` | pixels للـ replay |

---

## التنصيب

```
pip install -r requirements.txt
```

والـ Fairino SDK **مش على pypi** — إنسخ فولدر `fairino` من مشروع `cobot-cell`
القديم وحطه جنب `main.py`.
