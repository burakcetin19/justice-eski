"""
================================================================================
JUSTICE — Gazebo Simülasyon Launch Dosyası
================================================================================

GPS + Görsel Hibrit Takip Sistemi — Tek Komutla Başlat

Başlatılan Node'lar:
  1. gz_camera_bridge  — Gazebo kamera → ROS2 köprüsü
  2. tracker_node      — YOLO + TCTrack++ görsel takip
  3. justice_node      — Birleşik GPS(L1) + Görsel(PID) hibrit kontrol

⚠️  INSTANCE DAĞILIMI:
  ┌──────────────┬───────────┬──────┬──────────┬─────────┬──────────────────┐
  │ Rol          │ Instance  │ Port │ Model    │ Sys ID  │ DDS Namespace    │
  ├──────────────┼───────────┼──────┼──────────┼─────────┼──────────────────┤
  │ TAKİPÇİ     │ 0         │ 8888 │ 4007     │ 1       │ (yok) /fmu/...   │
  │ HEDEF        │ 1         │ 8889 │ 4003     │ 2       │ /px4_1/fmu/...   │
  └──────────────┴───────────┴──────┴──────────┴─────────┴──────────────────┘

Ön Koşullar (ayrı terminal'lerde başlatın):

  Terminal 1 — TAKİPÇİ Uçak (Instance 0, 4007, kameralı):
    cd ~/PX4-Autopilot
    PX4_GZ_WORLD=default PX4_SYS_AUTOSTART=4007 PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \\
      PX4_INSTANCE=0 PX4_UXRCE_DDS_PORT=8888 make px4_sitl gz_rc_cessna
    → ARM + takeoff yapın, justice offboard devralacak.

  Terminal 2 — HEDEF Uçak (Instance 1, 4003, kamerasız):
    cd ~/PX4-Autopilot
    PX4_GZ_WORLD=default PX4_SYS_AUTOSTART=4003 PX4_GZ_MODEL_POSE="10,0,0,0,0,0" \\
      PX4_INSTANCE=1 PX4_UXRCE_DDS_PORT=8889 ./build/px4_sitl_default/bin/px4 -i 1
    → QGC ile ARM + takeoff → ~50m irtifaya çıkarın.

  Terminal 3 — MicroXRCE-DDS Agent (Takipçi - port 8888):
    MicroXRCEAgent udp4 -p 8888

  Terminal 4 — MicroXRCE-DDS Agent (Hedef - port 8889):
    MicroXRCEAgent udp4 -p 8889

  Terminal 5 — Justice sistemi:
    ros2 launch justice justice_sim.launch.py

Başlatma Sırası:
  1. İki PX4 instance'ı başlatın (Terminal 1 + 2)
  2. DDS Agent'ları başlatın (Terminal 3 + 4)
  3. Hedef uçağı (Instance 1) QGC ile ~50m irtifaya uçurun
  4. Takipçi uçağı (Instance 0) ARM + takeoff yapın
  5. ros2 launch justice justice_sim.launch.py
  6. Justice otomatik offboard'a geçer ve hedefi takip eder

Argümanlar:
  px4_ns:      Takipçi PX4 namespace (varsayılan: '' = Instance 0)
  base_speed:  Temel ileri hız m/s (varsayılan: 15.0)
================================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():

    # ---- Launch Argümanları ----
    px4_ns_arg = DeclareLaunchArgument(
        'px4_ns', default_value='',
        description='Takipçi PX4 namespace (Instance 0 = boş)')

    base_speed_arg = DeclareLaunchArgument(
        'base_speed', default_value='15.0',
        description='Temel ileri hız (m/s)')

    # ---- ros_gz_image: Kamera Görüntüsü (Gazebo → ROS2) ----
    # Instance 0 (4007) kameralı model: rc_cessna_mono_cam
    # Gazebo'da model adı: rc_cessna_mono_cam_0
    # ⚠️ Kamera gelmezse kontrol:  gz topic -l | grep camera
    _gz_topic = '/world/default/model/rc_cessna_mono_cam_0/link/camera_link/sensor/camera/image'

    gz_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='gz_camera_bridge',
        output='screen',
        arguments=[_gz_topic],
        remappings=[
            (_gz_topic, '/sensor/camera/image_raw'),
        ],
    )

    # ---- Tracker Node (YOLO + TCTrack++) ----
    _tracking_dir = os.path.expanduser('~/TOY_SIHA/Ekip_calismalari/justice/justice/tracking')

    tracker = Node(
        package='justice',
        executable='tracker_node',
        name='tracker_node',
        output='screen',
        parameters=[{
            'yolo_model_path':     os.path.join(_tracking_dir, 'snapshot', 'best.pt'),
            'tctrack_config_path': os.path.join(_tracking_dir, 'experiments', 'TCTrack', 'config.yaml'),
            'tctrack_model_path':  os.path.join(_tracking_dir, 'snapshot', 'TCTrack.pth'),
        }],
    )

    # ---- Justice Node (GPS + Görsel Hibrit Kontrol) ----
    # TAKİPÇİ = Instance 0 → namespace yok, sys_id=1
    # HEDEF    = Instance 1 → namespace /px4_1
    justice = Node(
        package='justice',
        executable='justice_node',
        name='justice_node',
        output='screen',
        parameters=[{
            # PX4 — TAKİPÇİ = Instance 0
            'px4.namespace':           LaunchConfiguration('px4_ns'),  # '' → /fmu/in/... & /fmu/out/...
            'px4.mav_sys_id':          1,    # Instance 0 = system ID 1
            'px4.control_frequency':   50.0,

            # L1 Guidance
            'l1_guidance.l1_distance': 25.0,
            'l1_guidance.adaptive_l1': True,

            # Hedef Seçim Ağırlıkları
            'weights.distance':        0.25,
            'weights.angle':           0.30,
            'weights.speed':           0.15,
            'weights.camera_fov':      0.20,
            'weights.reachability':    0.10,

            # Uçak Limitleri
            'aircraft_limits.min_airspeed':    15.0,
            'aircraft_limits.max_airspeed':    35.0,
            'aircraft_limits.cruise_airspeed': 22.0,

            # Görsel PID
            'visual.kp_yaw':          0.002,
            'visual.ki_yaw':          0.0001,
            'visual.kd_yaw':          0.001,
            'visual.kp_pitch':        0.003,
            'visual.ki_pitch':        0.0001,
            'visual.kd_pitch':        0.001,
            'visual.base_speed':      LaunchConfiguration('base_speed'),
            'visual.max_yaw_rate':    0.5,
            'visual.max_vz':          3.0,
            'visual.min_speed':       10.0,
            'visual.max_speed':       25.0,

            # Hibrit Geçiş
            'transition.gps_to_visual_distance':     200.0,
            'transition.min_consecutive_detections': 5,
            'transition.visual_lost_timeout':        10.0,

            # HEDEF = Instance 1 → DDS namespace /px4_1
            'namespaces.targets':     '/px4_1',
        }],
    )

    return LaunchDescription([
        px4_ns_arg,
        base_speed_arg,
        gz_bridge,
        tracker,
        justice,
    ])