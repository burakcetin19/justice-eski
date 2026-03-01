# ⚔️ JUSTICE — GPS + Görsel Hibrit Takip Sistemi

**TEKNOFEST Savaşan İHA** yarışması için geliştirilmiş, **L1 Adaptive Guidance** (GPS) ve **YOLO + TCTrack++** (Görsel) tabanlı otonom sabit kanatlı İHA takip sistemi.

Drone, hedefe uzaktayken GPS koordinatları ile L1 Guidance algoritması üzerinden yaklaşır. Yeterince yaklaştığında kamera görüntüsünden YOLO ile hedefi tespit edip TCTrack++ ile takip ederek PID servolama ile görsel kilitleme gerçekleştirir.

---

## 📋 İçindekiler

- [Özellikler](#-özellikler)
- [Mimari](#-mimari)
- [Gereksinimler](#-gereksinimler)
- [Kurulum](#-kurulum)
- [Model Ağırlıklarının İndirilmesi](#-model-ağırlıklarının-indirilmesi)
- [Simülasyon Başlatma](#-simülasyon-başlatma)
- [Instance Dağılımı](#-instance-dağılımı)
- [Çalışma Prensibi](#-çalışma-prensibi)
- [Parametre Ayarları](#-parametre-ayarları)
- [Sorun Giderme](#-sorun-giderme)
- [Proje Yapısı](#-proje-yapısı)

---

## 🚀 Özellikler

- **L1 Adaptive Guidance**: GPS koordinatları üzerinden sabit kanatlı uçuş geometrisine uygun hedef takibi
- **YOLO Hedef Tespiti**: Gerçek zamanlı nesne tespiti ile hedef uçak algılama
- **TCTrack++ Takip**: Temporal-aware tracker ile kare-kare stabil takip
- **Hibrit Geçiş**: Mesafe + ardışık kamera tespitine dayalı otomatik GPS→Görsel geçişi
- **4 Saniye Kilitlenme Kuralı**: Hedef yeterince büyük ve merkezdeyse 4 saniye sayacı
- **Çoklu Hedef Seçimi**: Ağırlıklı skor sistemi (mesafe, açı, hız, FOV, ulaşılabilirlik)
- **PX4 Offboard Entegrasyonu**: micro-XRCE-DDS üzerinden tam otonom kontrol

---

## 🏗 Mimari

```
┌─────────────────────────────────────────────────────────┐
│                    JUSTICE SİSTEMİ                       │
├──────────────────┬──────────────────────────────────────┤
│                  │                                      │
│  tracker_node    │           justice_node                │
│  ┌────────────┐  │  ┌──────────────────────────────┐    │
│  │ YOLO Tespit│  │  │      DURUM MAKİNESİ          │    │
│  │     ↓      │  │  │                              │    │
│  │  TCTrack++ │──│──│→ BEKLEME → KALKIŞ_BEKLE      │    │
│  │   Takip    │  │  │    → OFFBOARD_GEÇİŞ          │    │
│  └────────────┘  │  │    → GPS_TAKİP (L1)          │    │
│        ↑         │  │    ⇄ GÖRSEL_TAKİP (PID)      │    │
│   Gazebo Kamera  │  │    → KİLİTLENME ✓            │    │
│   (ros_gz_image) │  └──────────────────────────────┘    │
│                  │         ↕ PX4 micro-XRCE-DDS         │
└──────────────────┴──────────────────────────────────────┘





<img width="1627" height="2296" alt="rosgraph" src="https://github.com/user-attachments/assets/f13188bc-7bd9-4475-b4be-a87d970824f7" />















```

**Durum Geçişleri:**

| Durum | Kontrol Modu | Açıklama |
|-------|-------------|----------|
| `BEKLEME` | — | PX4 bağlantısı bekleniyor |
| `KALKIŞ_BEKLE` | — | Manuel ARM bekleniyor |
| `OFFBOARD_GEÇİŞ` | — | PX4 Offboard moduna geçiş |
| `GPS_TAKİP` | Position Setpoint | L1 Guidance ile hedef GPS'e göre uçuş |
| `GÖRSEL_TAKİP` | Velocity Setpoint | PID ile kamera hatasına göre servolama |
| `KİLİTLENME` | Velocity Setpoint | 4s kilitlenme onaylandı, takip devam |

---

## 📦 Gereksinimler

### Sistem

| Bileşen | Versiyon |
|---------|---------|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| PX4 Autopilot | v1.14+ |
| Gazebo | Harmonic (Garden+) |
| Python | 3.10+ |
| CUDA | 11.7+ (GPU takip için) |

### Python Paketleri

```bash
pip install torch torchvision numpy opencv-python ultralytics
```

### ROS 2 Paketleri

```bash
sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs
```

### PX4 Mesajları

```bash
# px4_msgs paketini workspace'e klonlayın
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git -b release/1.14
cd ~/ros2_ws && colcon build --packages-select px4_msgs
source ~/ros2_ws/install/setup.bash
```

### Gazebo Kamera Köprüsü

```bash
# ros_gz_image paketi (Gazebo → ROS2 kamera köprüsü)
sudo apt install ros-humble-ros-gz-image
# veya kaynak koddan:
# https://github.com/gazebosim/ros_gz
```

### MicroXRCE-DDS Agent

```bash
# Snap ile kurulum (en kolay):
sudo snap install micro-xrce-dds-agent --edge

# veya kaynak koddan:
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
```

---

## 🔧 Kurulum

```bash
# 1. Repo'yu klonlayın
git clone https://github.com/burakcetin19/justice.git
cd justice

# 2. Model ağırlıklarını indirin (aşağıdaki bölüme bakın)

# 3. ROS 2 workspace'i derleyin
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash   # px4_msgs için
colcon build --packages-select justice

# 4. Workspace'i kaynak edin
source install/setup.bash
```

---

## 📥 Model Ağırlıklarının İndirilmesi

Model ağırlıkları boyutlarından dolayı (toplam ~134MB) repo'ya dahil değildir.
Aşağıdaki dosyaları indirip `justice/tracking/snapshot/` dizinine yerleştirin:

| Dosya | Boyut | Açıklama |
|-------|-------|----------|
| `best.pt` | ~20 MB | YOLO hedef tespit modeli |
| `TCTrack.pth` | ~114 MB | TCTrack++ takip modeli |

```bash
# Dosyaları indirdikten sonra:
cp best.pt  justice/tracking/snapshot/
cp TCTrack.pth justice/tracking/snapshot/

# Doğrulama:
ls -lh justice/tracking/snapshot/
# best.pt       ~20M
# TCTrack.pth   ~114M
# __init__.py
```

> **⚠️ Uyarı:** Bu dosyalar olmadan `tracker_node` başlatılamaz. Hata alırsanız dosya yollarını kontrol edin.

---

## 🎮 Simülasyon Başlatma

### Adım 0: PX4 SITL'ı derleyin (ilk seferde)

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_rc_cessna   # İlk build uzun sürebilir
# Çalışınca CTRL+C ile kapatabilirsiniz
```

### Adım 1: TAKİPÇİ Uçak (Instance 0, kameralı)

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=default \
PX4_SYS_AUTOSTART=4007 \
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
PX4_INSTANCE=0 \
PX4_UXRCE_DDS_PORT=8888 \
make px4_sitl gz_rc_cessna
```

PX4 konsolu (`pxh>`) açılınca DDS client durumunu doğrulayın:

```
pxh> uxrce_dds_client status
```

`running` yazmalıdır. Yazmazsa:

```
pxh> uxrce_dds_client start -t udp -p 8888
```

### Adım 2: HEDEF Uçak (Instance 1, kamerasız)

**Yeni bir terminal açın:**

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=default \
PX4_SYS_AUTOSTART=4003 \
PX4_GZ_MODEL_POSE="10,0,0,0,0,0" \
PX4_INSTANCE=1 \
PX4_UXRCE_DDS_PORT=8889 \
./build/px4_sitl_default/bin/px4 -i 1
```

DDS durumunu doğrulayın:

```
pxh> uxrce_dds_client status
```

### Adım 3: MicroXRCE-DDS Agent'ları Başlatın

**Her biri için ayrı terminal açın:**

```bash
# Terminal 3 — Takipçi Agent (port 8888)
MicroXRCEAgent udp4 -p 8888
```

```bash
# Terminal 4 — Hedef Agent (port 8889)
MicroXRCEAgent udp4 -p 8889
```

> **✅ Doğrulama:** Agent çalışırken `[info] Client connected` mesajı görmelisiniz. Görmüyorsanız PX4 instance'ların DDS port'unu kontrol edin.

### Adım 4: Hedef Uçağı Uçurun

**QGroundControl** ile Instance 1'e (hedef) bağlanın ve:
1. **ARM** edin
2. **Takeoff** yapın
3. ~50 metre irtifaya çıkarın
4. Manuel uçuşa devam edin (Mission veya Loiter modunda bırakabilirsiniz)

> **💡 İpucu:** QGC, birden fazla PX4 instance'ı otomatik algılar. Üst çubuktaki uçak simgesinden Instance 1'i seçin.

### Adım 5: Takipçi Uçağı Hazırlayın

Instance 0'ı (takipçi) QGC veya PX4 konsolundan:
1. **ARM** edin
2. **Takeoff** yapın ve irtifaya çıkmasını bekleyin

### Adım 6: Justice Sistemini Başlatın

**Yeni bir terminal açın:**

```bash
cd ~/TOY_SIHA/Ekip_calismalari/justice   # veya repo'nuzu klonladığınız dizin
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch justice justice_sim.launch.py
```

Justice otomatik olarak:
1. PX4'e bağlanır (`✅ PX4 BAĞLANDI!`)
2. ARM durumunu algılar (`KALKIŞ_BEKLE → OFFBOARD_GEÇİŞ`)
3. Offboard moduna geçer (`Offboard BAŞARILI!`)
4. GPS takibine başlar (`GPS_TAKİP (L1)`)
5. Hedef kamerada göründüğünde görsel takibe geçer (`GÖRSEL_TAKİP`)
6. 4 saniye kilitlenme sağlanınca onaylar (`🔒 KİLİTLENME ✓`)

---

## 📐 Instance Dağılımı

```
┌──────────────┬───────────┬──────┬───────────┬─────────┬──────────────────┐
│ Rol          │ Instance  │ Port │ Model ID  │ Sys ID  │ DDS Namespace    │
├──────────────┼───────────┼──────┼───────────┼─────────┼──────────────────┤
│ TAKİPÇİ     │ 0         │ 8888 │ 4007      │ 1       │ (yok) /fmu/...   │
│ HEDEF        │ 1         │ 8889 │ 4003      │ 2       │ /px4_1/fmu/...   │
└──────────────┴───────────┴──────┴───────────┴─────────┴──────────────────┘
```

### Justice Bağlantı Haritası

```
justice_node
  ├── Komut gönderir → /fmu/in/trajectory_setpoint     (Instance 0 — Takipçi)
  ├── Komut gönderir → /fmu/in/offboard_control_mode   (Instance 0 — Takipçi)
  ├── Kendi pozisyon okur ← /fmu/out/vehicle_local_position   (Instance 0)
  ├── Kendi GPS okur     ← /fmu/out/vehicle_global_position  (Instance 0)
  └── Hedef GPS okur     ← /px4_1/fmu/out/vehicle_global_position (Instance 1)

tracker_node
  └── Kamera okur ← /sensor/camera/image_raw (Gazebo → ros_gz_image köprüsü)
```

---

## ⚙️ Çalışma Prensibi

### GPS Takip (Uzak Mesafe)

```
Hedef GPS → geodetic_to_ned() → L1 Guidance → Position Setpoint → PX4
```

1. Hedefin GPS koordinatları `/px4_1/fmu/out/vehicle_global_position` üzerinden okunur
2. Koordinatlar NED (North-East-Down) frame'e dönüştürülür
3. L1 Guidance hedefin arkasında sanal bir nokta hesaplar
4. Bu noktaya Position Setpoint olarak gönderilir
5. PX4'ün kendi TECS/L1 kontrolcüsü uçağı bu noktaya yönlendirir

### Görsel Takip (Yakın Mesafe)

```
Kamera → YOLO → TCTrack++ → Piksel Hatası → PID → Velocity Setpoint → PX4
```

1. Gazebo kamerası → ros_gz_image → `/sensor/camera/image_raw`
2. YOLO ile hedef tespit edilir (bounding box)
3. TCTrack++ ile kare-kare takip sağlanır
4. Merkeze olan piksel hatası hesaplanır (x: yaw, y: pitch)
5. PID kontrolcüler yaw_rate ve dikey hız üretir
6. Body→NED dönüşümüyle Velocity Setpoint gönderilir

### GPS → Görsel Geçiş Koşulları

Üç koşul aynı anda sağlanmalıdır:

1. **Kamera hedef tespit etti** (`target_area > 0`)
2. **GPS mesafe < 200m** (ayarlanabilir)
3. **Ardışık tespit ≥ 5 kare** (kararlılık kontrolü)

---

## 🎛 Parametre Ayarları

Launch dosyasından veya `config/justice_params.yaml` üzerinden ayarlanabilir:

### L1 Guidance

| Parametre | Varsayılan | Açıklama |
|-----------|-----------|----------|
| `l1_guidance.l1_distance` | 25.0 | L1 referans mesafesi (m) |
| `l1_guidance.adaptive_l1` | true | Hıza göre adaptif L1 mesafesi |

### Uçak Limitleri

| Parametre | Varsayılan | Açıklama |
|-----------|-----------|----------|
| `aircraft_limits.min_airspeed` | 15.0 | Minimum hava hızı (m/s) |
| `aircraft_limits.max_airspeed` | 35.0 | Maksimum hava hızı (m/s) |
| `aircraft_limits.cruise_airspeed` | 22.0 | Seyir hızı (m/s) |

### Görsel PID

| Parametre | Varsayılan | Açıklama |
|-----------|-----------|----------|
| `visual.kp_yaw` | 0.002 | Yaw PID oransal kazanç |
| `visual.kp_pitch` | 0.003 | Pitch PID oransal kazanç |
| `visual.base_speed` | 15.0 | Görsel takipte ileri hız (m/s) |
| `visual.max_yaw_rate` | 0.5 | Maks yaw hızı (rad/s) |

### Hibrit Geçiş

| Parametre | Varsayılan | Açıklama |
|-----------|-----------|----------|
| `transition.gps_to_visual_distance` | 200.0 | Geçiş tetikleme mesafesi (m) |
| `transition.min_consecutive_detections` | 5 | Min ardışık kamera tespiti |
| `transition.visual_lost_timeout` | 10.0 | Görsel kayıp → GPS dönüş süresi (s) |

---

## 🔍 Sorun Giderme

### 1. PX4 Bağlantısı Kurulamıyor

**Belirti:** `⏳ PX4 bağlantısı yok!` veya `📡 PX4: Status:✗`

**Kontrol listesi:**

```bash
# DDS Agent çalışıyor mu?
ps aux | grep MicroXRCE

# Agent başlatılmamışsa:
MicroXRCEAgent udp4 -p 8888   # Takipçi için

# PX4 tarafında DDS client aktif mi?
# PX4 konsolunda (pxh>):
uxrce_dds_client status
# "running" yazmalı

# DDS client başlatılmamışsa:
uxrce_dds_client start -t udp -p 8888

# ROS2 topic'leri geliyorlar mı?
ros2 topic list | grep fmu
# Şunları görmelisiniz:
#   /fmu/out/vehicle_status
#   /fmu/out/vehicle_local_position
#   /fmu/out/vehicle_global_position
```

### 2. Hedef Verisi Gelmiyor (`hedef_sayı=0`)

**Belirti:** `L1=IDLE hedef=-1 mesafe=infm hedef_sayı=0`

**Kontrol listesi:**

```bash
# Instance 1'in (hedef) DDS topic'leri var mı?
ros2 topic list | grep px4_1
# Şunları görmelisiniz:
#   /px4_1/fmu/out/vehicle_local_position
#   /px4_1/fmu/out/vehicle_global_position

# Topic'te veri akıyor mu?
ros2 topic hz /px4_1/fmu/out/vehicle_global_position
# ~50 Hz civarında olmalı

# Eğer /px4_1/ topic'leri yoksa:
# → Instance 1'in DDS Agent'ı (port 8889) çalışmıyor olabilir
# → PX4 Instance 1 konsolunda:
uxrce_dds_client status
uxrce_dds_client start -t udp -p 8889
```

### 3. Instance ID'leri Karışık — Yanlış Uçak Kontrol Ediliyor

**Belirti:** Justice yanlış uçağı kontrol ediyor veya hedef olarak yanlış uçağı izliyor.

**Doğrulama:**

```bash
# Hangi instance hangi namespace'de?
ros2 topic echo /fmu/out/vehicle_status --once     # Instance 0 (takipçi)
ros2 topic echo /px4_1/fmu/out/vehicle_status --once  # Instance 1 (hedef)

# Her iki topic'ten de veri gelmeli.
# Eğer /fmu/out/... gelmiyor ama /px4_1/... geliyorsa:
#   → Instance numaraları ters olabilir
#   → PX4 başlatma komutlarındaki PX4_INSTANCE değerlerini kontrol edin
```

**Düzeltme:**
```
Takipçi: PX4_INSTANCE=0 PX4_UXRCE_DDS_PORT=8888
Hedef:   PX4_INSTANCE=1 PX4_UXRCE_DDS_PORT=8889
```

### 4. Kamera Görüntüsü Gelmiyor

**Belirti:** `tracker_node` çalışıyor ama `kamera görüntüsü bekleniyor...` diyor.

**Kontrol listesi:**

```bash
# Gazebo kamera topic'i var mı?
gz topic -l | grep camera
# Çıktıda model adını not edin, ör:
#   /world/default/model/rc_cessna_mono_cam_0/link/camera_link/sensor/camera/image

# ros_gz_image köprüsü çalışıyor mu?
ros2 topic hz /sensor/camera/image_raw
# 0 Hz ise köprü çalışmıyor demektir

# Gazebo topic adı launch dosyasındakiyle eşleşiyor mu?
# launch/justice_sim.launch.py dosyasındaki _gz_topic değişkenini
# gz topic -l çıktısıyla karşılaştırın.
# Eşleşmiyorsa launch dosyasını güncelleyin.
```

### 5. Offboard Moda Geçilemiyor

**Belirti:** `⏳ Offboard deneme #1 ... #2 ... #3 ...` tekrarlanıyor

**Sebepler:**
- Uçak ARM değil → Önce ARM edin
- Setpoint yeterince gönderilmedi → 10-15 saniye bekleyin
- DDS bağlantı kopukluğu → Agent'ları yeniden başlatın

```bash
# PX4 konsolunda manuel offboard geçiş:
commander mode offboard
```

### 6. Uçak Dümdüz İlerliyor, Hedefe Dönmüyor (Sabit Kanat)

**Sebepler:**
- PX4 sabit kanat kontrolcüsü Velocity Setpoint'te yatay dönüş yapamaz
- `mode=VEL` görüyorsanız bu normal beklenilen davranıştır (GPS uzak modunda)

**Çözüm:** `mode=POS` olduğunda uçak hedefe döner. Mesafe azaldıkça L1 Guidance Position Setpoint'e geçer.

### 7. Uçak İrtifayı Düşürüyor

**Sebepler:**
- Hedef uçak (Instance 1) havada değil veya düşük irtifada → NED `down ≈ 0` hesaplanır
- Instance karışıklığı → Yanlış uçağın verisini hedef olarak okuyor

**Çözüm:** Hedef uçağın gerçekten havada olduğunu doğrulayın:

```bash
ros2 topic echo /px4_1/fmu/out/vehicle_global_position --once | grep alt
# alt değeri yere göre yüksek olmalı (ör: 538.0 = 50m AGL)
```

### 8. `ModuleNotFoundError: px4_msgs`

```bash
# px4_msgs paketini workspace'e ekleyin ve kaynak edin:
source ~/ros2_ws/install/setup.bash
# veya .bashrc'ye ekleyin:
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## 📁 Proje Yapısı

```
justice/
├── .gitignore
├── package.xml                 # ROS2 paket tanımı
├── setup.py                    # Python paket kurulumu
├── setup.cfg
├── MANIFEST.in
├── README.md
│
├── launch/
│   └── justice_sim.launch.py   # Tek komutla başlatma
│
├── config/
│   └── justice_params.yaml     # Parametre dosyası
│
├── resource/
│   └── justice                 # ROS2 kaynak işaretçisi
│
└── justice/                    # Ana Python paketi
    ├── __init__.py
    ├── justice_node.py         # Ana kontrol node'u (durum makinesi + PX4)
    ├── l1_guidance.py          # L1 Guidance kütüphanesi (ROS2 bağımsız)
    ├── mock_target_receiver.py # Test için sahte hedef veri kaynağı
    │
    └── tracking/               # Görsel takip modülü
        ├── __init__.py
        ├── tracker_node.py     # YOLO + TCTrack++ ROS2 node'u
        ├── snapshot/           # Model ağırlıkları (.gitignore'da)
        │   ├── best.pt         # YOLO model (~20MB)
        │   └── TCTrack.pth     # TCTrack++ model (~114MB)
        ├── experiments/        # TCTrack konfigürasyonları
        │   ├── TCTrack/
        │   └── TCTrack++/
        └── pysot/              # TCTrack++ kaynak kodu
            ├── core/
            ├── datasets/
            ├── models/
            ├── tracker/
            └── utils/
```

---

## 🧑‍💻 Hızlı Kontrol Listesi

Simülasyonu başlatmadan önce bu listeyi takip edin:

- [ ] PX4 Autopilot derlenmiş (`make px4_sitl gz_rc_cessna`)
- [ ] Model ağırlıkları yerinde (`justice/tracking/snapshot/best.pt` + `TCTrack.pth`)
- [ ] `px4_msgs` paketi derlenmiş ve kaynak edilmiş
- [ ] `ros_gz_image` paketi kurulu
- [ ] `justice` paketi derlenmiş (`colcon build --packages-select justice`)
- [ ] Instance 0 (takipçi) başlatılmış — port 8888
- [ ] Instance 1 (hedef) başlatılmış — port 8889
- [ ] DDS Agent (port 8888) çalışıyor
- [ ] DDS Agent (port 8889) çalışıyor
- [ ] Her iki PX4'te `uxrce_dds_client status` → `running`
- [ ] `ros2 topic list | grep fmu` → topic'ler listeleniyor
- [ ] Hedef uçak ~50m irtifada uçuyor
- [ ] Takipçi uçak ARM + takeoff yapılmış


