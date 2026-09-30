# تشغيل beko_low_pressure على Jetson — JetPack 6.2

JetPack 6.2 = **Ubuntu 22.04 + Python 3.10 + CUDA 12.6** (aarch64). الكود هو هو؛ اللي
بيختلف عن ويندوز هو البيئة. **الترتيب مهم** — خصوصاً إن torch يتسطب قبل
`requirments.txt`.

وفي الآخر:

```bash
python3 check_linux.py
```

بيطبع سطر لكل حاجة: `ok` / `!!` (لازم تتصلح) / `--` (اختياري).

---

## 1. الجيتسون نفسه

```bash
sudo nvpmodel -m 0          # أعلى power mode (MAXN / MAXN SUPER حسب البورد)
sudo jetson_clocks          # الساعات على الآخر -- فرق واضح في سرعة الموديل
sudo apt update
sudo apt install -y python3-venv python3-pip git curl \
                    unixodbc unixodbc-dev libusb-1.0-0 libgl1
```

## 2. الـ virtualenv

```bash
cd beko_low_pressure
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install 'numpy<2'
```

`numpy<2` **قبل أي حاجة** — torch بتاع NVIDIA للجيتسون متبني على numpy 1.x.

## 3. torch + torchvision من NVIDIA (قبل requirments.txt)

من الـ index بتاع Jetson AI Lab (JetPack 6 / CUDA 12.6). على خطوتين عشان pip ميخلطش
بينه وبين pypi العادي (pypi فيه torch لـ aarch64 كمان بس **مش** للجيتسون):

```bash
pip download torch==2.8.0 torchvision==0.23.0 --no-deps -d ~/jetson-wheels \
    --index-url https://pypi.jetson-ai-lab.io/jp6/cu126
pip install ~/jetson-wheels/*.whl
```

اتأكد إنه شايف الـ GPU:

```bash
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
# لازم يطلع True -- لو False يبقى torch اتسطب من pypi العادي، امسحه وعيد
```

لو `import torch` اشتكى من `libcusparseLt.so` أو `libcudss`، ثبّت المكتبة الناقصة من
NVIDIA (cuSPARSELt / cuDSS لـ CUDA 12.6) وعيد.

⚠️ لو سطّبت `requirments.txt` **قبل** الخطوة دي، ultralytics هيجيب torch من pypi
**من غير CUDA الجيتسون** — الموديل هيشتغل على الـ CPU وبطيء جداً. الحل:
`pip uninstall torch torchvision` وعيد الخطوة 3.

## 4. باقي المكتبات

```bash
pip install -r requirments.txt
```

الملف فيه limits للجيتسون: `numpy<2` و `opencv-python<4.12` (4.12+ بيطلب numpy 2
وده بيكسر torch بتاع NVIDIA).

## 5. الـ RealSense

`pyrealsense2` بقى ليه wheel جاهز لـ aarch64 / Python 3.10 على pypi، فبيتسطب من
`requirments.txt` عادي. محتاج بس udev rules:

```bash
sudo curl -o /etc/udev/rules.d/99-realsense-libusb.rules \
  https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

افصل الكاميرا ووصّلها تاني في بورت **USB 3**. اختبار:

```bash
python3 -c "import pyrealsense2 as rs; print(len(rs.context().query_devices()), 'camera(s)')"
```

لو طلع 0 و `lsusb` شايفها: الـ rules، أو افصل ووصّل تاني. ولو الـ wheel نفسه
مش راضي يشتغل، البديل build من السورس بـ `-DFORCE_RSUSB_BACKEND=ON
-DBUILD_PYTHON_BINDINGS=bool:true`.

## 6. الـ SQL Server ODBC driver — `db.py`

ODBC Driver 18 بيدعم ARM64 على Ubuntu 22.04. على ويندوز `db.py` كان بيسطّب
`msodbcsql.msi` — ده مش موجود هنا، التسطيب من apt:

```bash
curl -sSL -O https://packages.microsoft.com/config/ubuntu/22.04/packages-microsoft-prod.deb
sudo dpkg -i packages-microsoft-prod.deb && rm packages-microsoft-prod.deb
sudo apt update
sudo ACCEPT_EULA=Y apt install -y msodbcsql18
python3 -c "import pyodbc; print(pyodbc.drivers())"   # لازم يطلع ODBC Driver 18 for SQL Server
```

لو `last_db1_settings.txt` / `last_db2_settings.txt` فيهم `Windows Authentication`
**مش هيشتغلوا على لينكس** — غيّرهم لـ SQL user / password.

## 7. الـ Fairino SDK — مفيش حاجة تتعمل

فولدر `fairino/` **جزء من المشروع** وموجود جنب `app.py` — مش محتاج تنسخه ولا
تدوّر على نسخة لينكس. الكود بيستخدم `fairino/Robot.py`، وده Python عادي (xmlrpc +
socket) بيشتغل على أي نظام وأي معالج، بما فيهم الجيتسون. اتجرب
`from fairino import Robot` على لينكس واشتغل.

فولدر `fairino/build/` فيه ملفات `.pyd` و `.so` — دي **بواقي build قديمة**
(`setup.py` بيعملها بـ Cython) ومحدش بيستخدمها: Python بيلاقي `Robot.py` الأول.
**متشغّلش** `python setup.py build_ext --inplace` على الجيتسون — مش محتاجه، وهيحط
نسخة compiled جنب `Robot.py` من غير أي فايدة.

## 8. الموديل

* انسخ ملف الـ weights. **أسماء الملفات case-sensitive**: `Weld.pt` ≠ `weld.pt`.
* للسرعة على الجيتسون حوّله لـ TensorRT **على الجيتسون نفسه** (مرة واحدة، بياخد
  دقايق):
  ```bash
  yolo export model=weld.pt format=engine half=True
  ```
  وبعدها في `cobot_kit/settings.py`: `"weights": "weld.engine"` — `AiModel` بيقراه
  زي الـ `.pt`. ملف الـ `.engine` مرتبط بالجهاز ونسخة TensorRT، متنقلوش من جهاز لجهاز.

## 9. الشبكة

الكوبوت على `192.168.57.2` — الجيتسون محتاج IP ثابت على نفس الـ subnet على
الكارت المتوصل بالروبوت:

```bash
nmcli con show
sudo nmcli con mod "Wired connection 1" ipv4.method manual ipv4.addresses 192.168.57.10/24
sudo nmcli con up "Wired connection 1"
ping 192.168.57.2
```

الـ Orin Nano Dev Kit فيه كارت Ethernet واحد — لو الروبوت والـ scanner والـ SQL
Server والفرونت على شبكات مختلفة هتحتاج USB-Ethernet adapter أو switch يجمعهم.

## 10. الملفات اللي مش في git

انسخ من جهاز الويندوز جنب `app.py`:

* `.env` (فيه `BOT_TOKEN` و `GROUP_CHAT_ID`)
* `last_db1_settings.txt` و `last_db2_settings.txt`
* ملف الـ weights

## 11. الشاشة

`ClickPicker` و `cv2.imshow` محتاجين شاشة. لو الجيتسون شغال headless والـ AI هو
اللي بيختار، مش محتاجهم.

## 12. UseePlus (لو هتستخدمها)

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2ce3", ATTR{idProduct}=="3828", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/99-useeplus.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
pip install pyusb libusb-package
```
