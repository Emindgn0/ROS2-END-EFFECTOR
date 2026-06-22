# Doosan H2515 — B-Pillar Zımparalama End-Effector Sistemi
## Proje Dökümantasyonu

---

## İçindekiler

1. [Sistem Genel Bakış](#1-sistem-genel-bakış)
2. [Donanım Bağlantıları](#2-donanım-bağlantıları)
3. [Windows / WSL2 Kurulumu](#3-windows--wsl2-kurulumu)
   - USB Seri Port (CAN Kart) İzinleri
   - USB Kamera İzinleri
4. [Projeyi Derleme](#4-projeyi-derleme)
5. [Başlatma Yöntemleri](#5-başlatma-yöntemleri)
   - Masaüstü İkonu (start_robot.py)
   - Terminal ile Manuel
6. [ROS2 Düğümleri — Ne İş Yapar?](#6-ros2-düğümleri--ne-i̇ş-yapar)
   - can_node
   - gui_node
   - logic_node
   - vision_node
7. [ROS2 Topic'leri](#7-ros2-topicler)
8. [CAN Protokolü](#8-can-protokolü)
9. [Çalışma Modları](#9-çalışma-modları)
10. [Sık Karşılaşılan Sorunlar](#10-sık-karşılaşılan-sorunlar)

---

## 1. Sistem Genel Bakış

Bu sistem, Doosan H2515 robot koluna takılı bir end-effector'ü kontrol eder. End-effector üzerinde:

- **2 adet servo motor** — Pan (S1) ve Tilt (S2) eksenleri, kamera kutusunu hareket ettirir
- **1 adet röle** — Zımpara motorunu açıp kapatır
- **4 adet load cell (yük hücresi)** — Temas kuvvetini Newton cinsinden ölçer
- **USB kamera (OsmoAction4)** — B-pillar yüzeyini tarar, YOLO ile çapak tespit eder

Tüm bu donanım bir **CAN kartı** üzerinden USB-seri (CH340 çipi) bağlantısıyla bilgisayara bağlıdır.

```
Bilgisayar (WSL2)
      │
      ├─ USB ─→ CAN Kartı (CH340, /dev/ttyUSB0, 2Mbaud)
      │              ├─ Servo S1 (Pan)
      │              ├─ Servo S2 (Tilt)
      │              ├─ Röle (Zımpara)
      │              └─ 4x Load Cell
      │
      └─ USB ─→ OsmoAction4 Kamera (/dev/video0)
```

**Yazılım Katmanı:**

```
ROS2 Humble (Ubuntu 22.04 / WSL2)
   ├─ can_node    — CAN kartıyla haberleşir, load cell yayınlar
   ├─ vision_node — Kamera, YOLO çapak tespiti
   ├─ logic_node  — Otonom görev sıralaması
   └─ gui_node    — PyQt6 operatör arayüzü
```

---

## 2. Donanım Bağlantıları

| Donanım | Bağlantı | WSL2'deki Adres |
|---|---|---|
| CAN Kart (CH340) | USB ↔ PC | `/dev/ttyUSB0` |
| OsmoAction4 Kamera | USB ↔ PC | `/dev/video0` (veya video2) |
| Doosan H2515 | Ethernet ↔ PC | `192.168.137.100` (varsayılan) |

**Not:** CAN kartı ile kamera her ikisi de USB olduğu için `/dev/ttyUSB0` sırası değişebilir.  
Hangi aygıt olduğunu anlamak için:
```bash
ls -la /dev/ttyUSB*
# veya
dmesg | grep -i "ttyUSB\|ch340" | tail -10
```

---

## 3. Windows / WSL2 Kurulumu

### 3.1 USB Seri Port İzinleri (CAN Kart)

WSL2'de USB aygıtları varsayılan olarak bağlı gelmez. İki yöntem vardır:

#### Yöntem A — Her Seferinde Manuel (Geçici)

**Windows Tarafı (PowerShell — Yönetici olarak):**
```powershell
# usbipd kurulu değilse: winget install --interactive --exact dorssel.usbipd-win
usbipd list                          # USB aygıtlarını listele, BUSID'yi bul
usbipd attach --wsl --busid <BUSID>  # Örn: usbipd attach --wsl --busid 2-1
```

**WSL2 Ubuntu Tarafı:**
```bash
# Aygıt geldi mi kontrol et
ls /dev/ttyUSB*

# İzin ver (her yeniden bağlamada tekrar gerekir)
sudo chmod 666 /dev/ttyUSB0
```

#### Yöntem B — Kalıcı Udev Kuralı (Önerilen)

Bu yöntemde `/dev/ttyUSB0` her geldiğinde otomatik olarak herkes tarafından okunabilir olur:

```bash
# Kural dosyasını oluştur
sudo nano /etc/udev/rules.d/99-usb-serial.rules
```

Dosyaya şunu yaz:
```
KERNEL=="ttyUSB*", MODE="0666"
```

Kaydet, çık. Kuralları yenile:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Sonraki USB bağlamalarında `chmod` gerekmez.

### 3.2 USB Kamera İzinleri

```bash
# Kamera aygıtına izin ver
sudo chmod 777 /dev/video0
sudo chmod 777 /dev/video1   # varsa

# Kalıcı çözüm — video grubu için udev kuralı:
sudo nano /etc/udev/rules.d/99-camera.rules
```

Dosyaya şunu yaz:
```
KERNEL=="video*", MODE="0666"
```

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 3.3 usbipd — Windows USB Yönlendirme Aracı

WSL2 doğal olarak USB aygıtlarını görmez. `usbipd-win` bu sorunu çözer.

**Kurulum (bir kere yapılır):**
```powershell
# Windows PowerShell (Yönetici)
winget install --interactive --exact dorssel.usbipd-win
```

**Her oturumda (CAN kart + kamera için):**
```powershell
# PowerShell (Yönetici)
usbipd list
# Çıktıda CH340 ve kameranın BUSID'sini bul, sonra:
usbipd attach --wsl --busid 2-1   # CAN kart için
usbipd attach --wsl --busid 2-3   # Kamera için (BUSID değişebilir)
```

**WSL2'de kontrol:**
```bash
ls /dev/ttyUSB*   # → /dev/ttyUSB0 görünmeli
ls /dev/video*    # → /dev/video0 görünmeli
```

### 3.4 WSL2 GUI Desteği (PyQt6 için)

WSL2'de GUI uygulamaları (PyQt6) çalıştırmak için Windows 11 veya WSLg gerekir.

```bash
# Test et
echo $DISPLAY   # → :0 veya benzeri bir şey çıkmalı

# Sorun varsa — Windows 10 için VcXsrv kurulabilir
# Windows 11 + WSLg için ekstra kurulum gerekmez
```

---

## 4. Projeyi Derleme

```bash
# Terminali aç, proje dizinine git
cd ~/ros2-end-effector

# ROS2 ortamını yükle
source /opt/ros/humble/setup.bash

# Derle
colcon build --symlink-install

# Yeni terminal her açıldığında:
source ~/ros2-end-effector/install/setup.bash
```

**Not:** `--symlink-install` sayesinde Python dosyalarında yapılan değişiklikler yeniden derleme gerektirmez.

---

## 5. Başlatma Yöntemleri

### 5.0 İkon Çalışmazsa — Terminal'den Başlatma

İkon çalışmıyorsa veya hata veriyorsa WSL2 terminalinden doğrudan çalıştır:

**1. WSL2 terminalini aç** (Windows'ta "Ubuntu" veya "WSL" arama yap)

**2. Ortamı hazırla ve ikonu terminalden çalıştır:**
```bash
cd ~/ros2-end-effector
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 start_robot.py
```

Bu komut masaüstü ikonuyla tamamen aynı şeyi yapar — aynı mod seçim penceresi açılır.

**3. Veya doğrudan launch komutu (pencere olmadan):**

```bash
# CAN Donanım modu (en sık kullanılan):
cd ~/ros2-end-effector
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch end_effector_ros2 end_effector.launch.py simulation:=false use_real_robot:=false
```

```bash
# Simülasyon modu:
cd ~/ros2-end-effector
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch end_effector_ros2 end_effector.launch.py simulation:=true use_real_robot:=false
```

**Tek satırlık kopyala-yapıştır versiyonları:**

```bash
# CAN Donanım
cd ~/ros2-end-effector && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch end_effector_ros2 end_effector.launch.py simulation:=false use_real_robot:=false
```

```bash
# Simülasyon
cd ~/ros2-end-effector && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch end_effector_ros2 end_effector.launch.py simulation:=true use_real_robot:=false
```

**Not:** Her yeni terminal açıldığında `source` komutlarını tekrar çalıştırmak gerekir.  
Bunu otomatik yapmak için `~/.bashrc` dosyasına ekleyebilirsin:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2-end-effector/install/setup.bash" >> ~/.bashrc
```
Bundan sonra her terminalde otomatik yüklenir, `source` yazmana gerek kalmaz.

---

### 5.1 Masaüstü İkonu (start_robot.py)

Masaüstüne yerleştirilen ikon tıklandığında `start_robot.py` çalışır ve bir mod seçim penceresi açılır:

```
┌─────────────────────────────────────────────────┐
│   🤖  Doosan H2515 — B-Pillar Zımparalama       │
├─────────────────────────────────────────────────┤
│  Robot IP Adresi (sadece Gerçek Robot için):    │
│  [ 192.168.137.100                            ] │
│                                                 │
│  CAN Donanım: USB seri kart + kamera — DSR yok │
│                                                 │
│  [Simülasyon] [CAN Donanım ▶] [Gerçek Robot]   │
└─────────────────────────────────────────────────┘
```

#### CAN Donanım Modu (Orta Buton — Varsayılan)
- Doosan robot kolu **bağlı değil**, sadece CAN kart ve kamera kullanılır
- `simulation:=false` ile başlar → CAN kartına gerçek komutlar gider
- En sık kullanılan mod

#### Simülasyon Modu (Sol Buton)
- Doosan DSR_ROBOT2 emülatörü başlatılır (Rviz ile)
- CAN donanımı olmadan load cell verisi simüle edilir
- Test ve geliştirme için

#### Gerçek Robot Modu (Sağ Buton)
- Doosan H2515 gerçek robota Ethernet ile bağlanır
- IP adresi girilmesi gerekir (varsayılan: `192.168.137.100`)
- Hem CAN kart hem robot kolu aktif

### 5.2 Terminal ile Manuel Başlatma

**Ortamı hazırla (her terminal için):**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2-end-effector/install/setup.bash
```

**CAN Donanım Modu:**
```bash
ros2 launch end_effector_ros2 end_effector.launch.py \
    simulation:=false use_real_robot:=false
```

**Simülasyon Modu:**
```bash
ros2 launch end_effector_ros2 end_effector.launch.py \
    simulation:=true use_real_robot:=false use_gazebo:=false
```

**Gerçek Robot Modu:**
```bash
# Önce DSR_ROBOT2 başlat (ayrı terminal)
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
    model:=h2515 mode:=real host:=192.168.137.100

# Sonra end-effector sistemi başlat
ros2 launch end_effector_ros2 end_effector.launch.py \
    simulation:=false use_real_robot:=true
```

**Ek Parametreler:**

| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `simulation` | `false` | `true` = CAN olmadan simüle et |
| `use_real_robot` | `false` | `true` = DSR_ROBOT2 gerçek robot hareketi |
| `can_port` | `/dev/ttyUSB0` | CAN kartının seri port yolu |
| `baudrate` | `2000000` | CAN baud hızı (2 Mbaud) |
| `camera_index` | `0` | USB kamera indeksi (0, 2 denenilebilir) |
| `model_name` | `YOLO26s.pt` | YOLO model dosyası |
| `stream_fps` | `30` | Kamera FPS |

**Örnek — farklı kamera indeksi:**
```bash
ros2 launch end_effector_ros2 end_effector.launch.py \
    simulation:=false camera_index:=2
```

---

## 6. ROS2 Düğümleri — Ne İş Yapar?

### 6.1 `can_node` (can_node.py)

**Görev:** CAN kart ile fiziksel haberleşme köprüsü.

**Ne yapar:**
- `/dev/ttyUSB0` portuna 2 Mbaud hızında bağlanır
- Servo komutlarını (`servo_command` topic) alıp CAN frame formatına çevirip gönderir
- Zımpara röle komutlarını (`sander_only` topic) alıp gönderir
- Load cell paketlerini okuyup `load_cells` topic'ine yayınlar
- CAN bağlantı durumunu `can_status` topic'ine yayınlar

**CAN TX frame formatı (10 byte):**
```
AA C5 03 03 00 [S1] [S2] [SANDER] 00 55
```
- S1: Pan servo açısı (0–180)
- S2: Tilt servo açısı (0–180)
- SANDER: 111 = Açık, 222 = Kapalı

**Load cell paketi (9 byte, CAN'dan alınan):**
```
AA C4 01 01 [LC1] [LC2] [LC3] [LC4] 55
```

**Parametreler:**

| Parametre | Değer |
|---|---|
| port | `/dev/ttyUSB0` |
| baudrate | `2000000` |
| simulation | `false` (gerçek mod) |
| use_dsr2 | `False` (kritik — True olursa spin thread bloke olur) |

### 6.2 `gui_node` (gui_node.py)

**Görev:** Operatör kontrol arayüzü — PyQt6 penceresi.

**Ne yapar:**
- Servo sliderları ve preset butonlarıyla S1/S2 açılarını manuel ayarlar
- "Sander ON / OFF" butonlarıyla zımpara rölesini kontrol eder
- "Camera Box OPEN/CLOSE" butonlarıyla S1=S2=35° (açık) veya S1=S2=160° (kapalı) gönderir
- "START AUTONOMOUS" ile otonom görevi başlatır
- "⚠ EMERGENCY STOP" ile tüm sistemi acil durdurur
- Load cell verilerini progress bar ile gösterir
- Kamera görüntüsünü YOLO annotation'larıyla gösterir
- Log mesajlarını ekranda gösterir
- CAN bağlantı durumunu gösterir ("Connected ✓" veya "Disconnected")

**Önemli:** GUI kapandığında tüm ROS2 sistemi kapanır (launch event handler).

### 6.3 `logic_node` (logic_node.py)

**Görev:** Otonom görev sıralaması ve yönlendirme.

**Otonom görev adımları:**

```
1. TARAMA POZİSYONU  → S1=S2=160° gönder, zımpara KAPALI
2. TARAMA            → 5 saniye YOLO çapak verisi topla
3. SERVO KAP         → S1=S2=kapalı pozisyon
4. Her çapak için:
   a. HOVER          → Robot çapak üzerine XY hizala
   b. Z İNİŞ         → Load cell 25N olana kadar aşağı in
   c. ZIMPARALA       → Güven seviyesine göre 2–8 saniye zımparala
   d. GERİ ÇEK        → Robot güvenli yüksekliğe geri çekil
5. GÖREV TAMAM       → Home pozisyona dön
```

**Kuvvet eşikleri:**
- `FORCE_CONTACT_THRESHOLD = 25.0 N` — Temas kabul edildi
- `FORCE_SAFETY_LIMIT = 50.0 N` — Acil geri çekilme

**Parametreler:**

| Parametre | Değer |
|---|---|
| simulation | `false` |
| use_real_robot | `false` (Gazebo IK) veya `true` (DSR_ROBOT2 movel) |

### 6.4 `vision_node` (vision_node.py)

**Görev:** Kamera görüntüsü alıp YOLO ile çapak tespit eder.

**Ne yapar:**
- USB kamerayı (`/dev/video0`) açar
- Her kareyi YOLO modeline verir
- Tespit edilen çapakları `detections` topic'ine yayınlar (JSON)
- Annotated görüntüyü `camera/image_annotated` topic'ine yayınlar
- Ham görüntüyü `camera/image_raw` topic'ine yayınlar

---

## 7. ROS2 Topic'leri

### Komut Topic'leri (GUI/Logic → CAN)

| Topic | Tip | Açıklama |
|---|---|---|
| `/end_effector/servo_command` | `String` (JSON) | Servo açıları: `{"s1":160,"s2":160,"sander":222}` veya kamera kutusu: `{"camera":0.025}` |
| `/end_effector/sander_only` | `String` (JSON) | Sadece zımpara: `{"sander":111}` (ON) veya `{"sander":222}` (OFF) |
| `/end_effector/mission_start` | `Bool` | `true` → Otonom görevi başlat |
| `/end_effector/mission_stop` | `Bool` | `true` → Görevi durdur |
| `/end_effector/emergency_stop` | `Bool` | `true` → Acil durdurma |
| `/end_effector/set_mode` | `String` | `"simulation"` veya `"hardware"` |

### Durum Topic'leri (CAN/Logic/Vision → GUI)

| Topic | Tip | Açıklama |
|---|---|---|
| `/end_effector/can_status` | `Bool` | CAN bağlantı durumu |
| `/end_effector/load_cells` | `String` (JSON) | `{"values":[F1,F2,F3,F4]}` Newton cinsinden |
| `/end_effector/servo_state` | `String` (JSON) | Son gönderilen servo/sander değerleri |
| `/end_effector/mission_status` | `String` (JSON) | Görev durumu, kuvvet, çapak sayısı |
| `/end_effector/guidance` | `String` (JSON) | Yönlendirme verisi: `{"dir":"TARANIYIOR... 3s","dx":0,"dy":0}` |
| `/end_effector/log` | `String` | Log mesajları |
| `/end_effector/detections` | `String` (JSON) | `{"burrs":[{"x":320,"y":240,"conf":0.85,"dist":0.5}]}` |
| `/end_effector/camera/image_raw` | `sensor_msgs/Image` | Ham kamera görüntüsü |
| `/end_effector/camera/image_annotated` | `sensor_msgs/Image` | YOLO kutularıyla işlenmiş görüntü |

### Terminalde Topic İzleme

```bash
# CAN bağlantı durumu
ros2 topic echo /end_effector/can_status

# Load cell kuvvet verisi
ros2 topic echo /end_effector/load_cells

# Görev durumu
ros2 topic echo /end_effector/mission_status

# Tüm topic'leri listele
ros2 topic list | grep end_effector
```

---

## 8. CAN Protokolü

### Başlatma Paketi (PC → CAN Kart)

CAN kart bağlandığında bir kere gönderilir (20 byte):
```
AA 55 12 07 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 1A
```

### Komut Frame'i (PC → CAN Kart, 10 byte)

```
AA  C5  03  03  00  [S1]  [S2]  [SANDER]  00  55
```

| Byte | Değer | Açıklama |
|---|---|---|
| 0 | `0xAA` | Header |
| 1 | `0xC5` | Komut tipi |
| 2 | `0x03` | Sabit |
| 3 | `0x03` | Sabit |
| 4 | `0x00` | Sabit |
| 5 | 0–180 | S1 Pan servo açısı |
| 6 | 0–180 | S2 Tilt servo açısı |
| 7 | 111/222 | Zımpara: 111=ON, 222=OFF |
| 8 | `0x00` | Sabit |
| 9 | `0x55` | Footer |

### Load Cell Paketi (CAN Kart → PC, 9 byte)

```
AA  C4  01  01  [LC1]  [LC2]  [LC3]  [LC4]  55
```

- LC değerleri: `ham_byte - 80` = Newton değeri (minimum -10.0N)

### Servo Pozisyon Referansları

| Pozisyon | S1 | S2 | Kullanım |
|---|---|---|---|
| Merkez (park) | 160° | 160° | Varsayılan, tarama başlangıcı |
| Kamera kutusu AÇIK | 35° | 35° | Kamera dışarı çıkmış |
| Kamera kutusu KAPALI | 160° | 160° | Kamera içeride, güvenli |

---

## 9. Çalışma Modları

### Mod 1: CAN Donanım (Günlük Kullanım)

```
simulation:=false  use_real_robot:=false
```

- CAN kart bağlı, robot kolu YOK
- Servo ve röle komutları fiziksel donanıma gider
- Load cell gerçek verisi okunur
- Robot kolu hareketi simüle edilir (Gazebo IK)

### Mod 2: Simülasyon (Test)

```
simulation:=true  use_real_robot:=false
```

- CAN kart BAĞLI OLMAK ZORUNDA DEĞİL
- Load cell verisi yazılımda simüle edilir (sinüs dalgası + gürültü)
- Servo komutları loglanır ama donanıma gitmez
- GUI'de CAN durumu "bağlı" görünmez (beklenen)

### Mod 3: Gerçek Robot (Full Sistem)

```
simulation:=false  use_real_robot:=true
```

- CAN kart + Doosan H2515 robot kolu birlikte
- DSR_ROBOT2 `move_line` servisleri kullanılır
- Ethernet üzerinden robot IP'sine bağlanır
- Tam otonom zımparalama

---

## 10. Sık Karşılaşılan Sorunlar

### "CAN: Disconnected" — Bağlantı Yok

```bash
# 1. Aygıt görünüyor mu?
ls /dev/ttyUSB*

# Yoksa: Windows'ta usbipd ile yeniden bağla
# PowerShell (Yönetici): usbipd attach --wsl --busid <ID>

# 2. İzin sorunu mu?
sudo chmod 666 /dev/ttyUSB0

# 3. Kalıcı çözüm:
echo 'KERNEL=="ttyUSB*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-usb-serial.rules
sudo udevadm control --reload-rules
```

### Kamera Görüntüsü Gelmiyor

```bash
# Kamera aygıtı var mı?
ls /dev/video*

# İzin ver
sudo chmod 777 /dev/video0

# Hangi indeks doğru? (0, 2 dene)
ros2 launch end_effector_ros2 end_effector.launch.py camera_index:=2

# v4l2 ile test
v4l2-ctl --list-devices
```

### Röle Tık Sesi Yok (Sander Komutu Gitmiyor)

Bu sorunun en sık sebebi `use_dsr2:=True` parametresidir. Launch dosyasında:
```python
'use_dsr2': False,   # Bu False olmak ZORUNDA
```
`True` olursa spin thread bloke olur, hiçbir komut geçmez.

**Kontrol:**
```bash
ros2 topic echo /end_effector/can_status   # True görünmeli
ros2 topic echo /end_effector/sander_only  # Komut gelince veri görünmeli
```

### "waiting for CAN..." Devam Ediyor

GUI `simulation:=true` ile başlatıldıysa CAN durumu gösterilmez (beklenen davranış).  
`simulation:=false` ile başlatıldıysa CAN bağlantısı kurulamazsa gösterilir — kablo bağlantısını ve usbipd'yi kontrol et.

### Otonom Modda Servo Hareket Etmiyor

1. `can_status` True mu? → `/end_effector/can_status` echo et
2. Logic node loglarına bak → `ros2 topic echo /end_effector/log`
3. CAN TX logu çıkıyor mu? → can_node terminalinde `CAN TX →` satırı görünmeli

### PyQt6 Penceresi Açılmıyor

```bash
# DISPLAY ayarı var mı?
echo $DISPLAY

# Yoksa
export DISPLAY=:0

# WSL2'de XDG_RUNTIME_DIR
export XDG_RUNTIME_DIR=/run/user/$(id -u)
```

### colcon build Hatası

```bash
# Önce ROS2 ortamını kaynak yap
source /opt/ros/humble/setup.bash

# Sonra derle
cd ~/ros2-end-effector
colcon build --symlink-install

# Bağımlılık eksikse
rosdep install --from-paths src --ignore-src -r -y
```

---

## Hızlı Referans Kartı

```bash
# == ORTAM ==
source /opt/ros/humble/setup.bash
source ~/ros2-end-effector/install/setup.bash

# == USB İZİNLER (her oturumda) ==
sudo chmod 666 /dev/ttyUSB0
sudo chmod 777 /dev/video0

# == BAŞLAT ==
# CAN Donanım:
ros2 launch end_effector_ros2 end_effector.launch.py simulation:=false

# Simülasyon:
ros2 launch end_effector_ros2 end_effector.launch.py simulation:=true

# == İZLE ==
ros2 topic echo /end_effector/can_status
ros2 topic echo /end_effector/load_cells
ros2 topic echo /end_effector/log

# == MANUEL KOMUT (test için) ==
# Sander ON:
ros2 topic pub --once /end_effector/sander_only std_msgs/msg/String \
    '{"data": "{\"sander\": 111}"}'

# Servo S1=90° S2=90°:
ros2 topic pub --once /end_effector/servo_command std_msgs/msg/String \
    '{"data": "{\"s1\": 90, \"s2\": 90}"}'

# Kamera kutusu AÇ:
ros2 topic pub --once /end_effector/servo_command std_msgs/msg/String \
    '{"data": "{\"camera\": 0.025}"}'
```

---

*Doosan H2515 End-Effector Sistemi — Proje Dökümantasyonu*
