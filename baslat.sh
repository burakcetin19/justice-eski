#!/bin/bash

echo "🚀 KGM TOY - TEKNOFEST Çoklu Uçuş Simülasyonu Başlatılıyor..."

# Çıkış yapıldığında arkada asılı işlem kalmaması için temizleyici
trap 'echo -e "\n🛑 Sistem Kapatılıyor..."; pkill -P $$; exit 0' SIGINT

# Projenin ana dizinini dinamik olarak al
REPO_ROOT=$(pwd)

# Modellerin yolunu Gazebo'ya tanıt
export GZ_SIM_RESOURCE_PATH=$REPO_ROOT/simulation/models:$GZ_SIM_RESOURCE_PATH

# Kendi dünyamızı PX4'ün klasörüne kısayol olarak bağla (Kullanıcı adından bağımsız)
ln -sf $REPO_ROOT/simulation/worlds/teknofest_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/teknofest_world.sdf

# 1. Micro XRCE-DDS Agent (UDP 8888)
echo "📡 Micro XRCE-DDS Agent Başlatılıyor..."
cd ~/Micro-XRCE-DDS-Agent/build
./MicroXRCEAgent udp4 -p 8888 &
sleep 2

# Erzurum Havalimanı Koordinatları
export PX4_HOME_LAT=39.9555
export PX4_HOME_LON=41.1719
export PX4_HOME_ALT=1757.0

# 2. Gazebo Harmonic Başlatılıyor
echo "🌍 Gazebo Simülasyonu (Erzurum) Yükleniyor..."
gz sim -r $REPO_ROOT/simulation/worlds/teknofest_world.sdf &
sleep 6

# 3. PX4 Çekirdeklerini (FCU) Ayrı Ayrı Başlatma
cd ~/PX4-Autopilot
export PX4_SIM_MODEL=rc_cessna

echo "✈️ 1. Otopilot (Takip Eden Uçak) Başlatılıyor... [SysID: 1]"
PX4_SYS_AUTOSTART=4003 ./build/px4_sitl_default/bin/px4 -i 0 -d ROMFS/px4fmu_common -s etc/init.d-posix/rcS > /dev/null 2>&1 &
sleep 2

echo "🎯 2. Otopilot (Hedef Uçak) Başlatılıyor... [SysID: 2]"
PX4_SYS_AUTOSTART=4003 ./build/px4_sitl_default/bin/px4 -i 1 -d ROMFS/px4fmu_common -s etc/init.d-posix/rcS > /dev/null 2>&1 &

echo "✅ Fizik motoru ve Otopilotlar Aktif!"
echo "👉 Şimdi QGroundControl'ü açabilir ve diğer terminalde 'ros2 launch justice justice_sim.launch.py' komutunu çalıştırabilirsiniz."
echo "🛑 Kapatmak için bu terminalde Ctrl+C yapman yeterli."

wait