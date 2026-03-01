#!/usr/bin/env python3
"""
================================================================================
MOCK TARGET RECEIVER NODE
================================================================================
Mock sunucudan WebSocket üzerinden hedef uçak telemetri verilerini alır ve
gps_tracking_node'un beklediği formatlarda ROS2 topic'lere yayınlar.

Veri Akışı:
  Mock Sunucu (ws://localhost:8080)
    → telemetry event
    → Bu Node
    → ROS2 Topics → gps_tracking_node

Topics Published (gps_tracking_node uyumlu):
  /competition/target_list (Float64MultiArray): [id, lat, lon, alt, speed, heading] x N
  /simulation/enemy_aircraft (String): JSON formatında hedef listesi
  /mock/target_count (Int32): Aktif hedef sayısı

Kullanım:
  ros2 run teknofest_control mock_target_receiver --ros-args -p mock_server_url:=http://localhost:8080
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import socketio
import json
import time
import threading
from typing import Dict, Optional
from dataclasses import dataclass, asdict

# ROS2 mesaj tipleri
from std_msgs.msg import String, Float64MultiArray, Int32


@dataclass
class TargetState:
    """Hedef uçak durumu"""
    id: str
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw: float = 0.0
    speed: float = 0.0
    timestamp: float = 0.0
    status: str = "UNKNOWN"
    last_update: float = 0.0


class MockTargetReceiver(Node):
    """
    Mock sunucudan hedef uçak verilerini alan ROS2 node'u.
    WebSocket üzerinden gelen telemetri verilerini ROS2 topic'lere yayınlar.
    """

    def __init__(self):
        super().__init__('mock_target_receiver')

        # =====================================================================
        # PARAMETRELER
        # =====================================================================
        self.declare_parameter('mock_server_url', 'http://localhost:8080')
        self.declare_parameter('target_timeout_sec', 5.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('own_uav_id', 'bizim_iha')
        self.declare_parameter('auto_reconnect', True)
        self.declare_parameter('reconnect_delay_sec', 3.0)

        self.mock_server_url = self.get_parameter('mock_server_url').value
        self.target_timeout = self.get_parameter('target_timeout_sec').value
        self.publish_rate = self.get_parameter('publish_rate_hz').value
        self.own_uav_id = self.get_parameter('own_uav_id').value
        self.auto_reconnect = self.get_parameter('auto_reconnect').value
        self.reconnect_delay = self.get_parameter('reconnect_delay_sec').value

        # =====================================================================
        # HEDEF TAKİP DEĞİŞKENLERİ
        # =====================================================================
        self.targets: Dict[str, TargetState] = {}
        self.targets_lock = threading.Lock()
        self.connected = False

        # =====================================================================
        # QoS PROFİLİ
        # =====================================================================
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # =====================================================================
        # PUBLISHER'LAR (gps_tracking_node uyumlu)
        # =====================================================================
        # Float64MultiArray formatı: [id, lat, lon, alt, speed, heading] x N
        self.target_list_pub = self.create_publisher(
            Float64MultiArray, '/competition/target_list', qos
        )

        # JSON formatı (alternatif)
        self.enemy_aircraft_pub = self.create_publisher(
            String, '/simulation/enemy_aircraft', qos
        )

        # Hedef sayısı
        self.target_count_pub = self.create_publisher(
            Int32, '/mock/target_count', qos
        )

        # Hedef ID sayacı (string ID'leri integer'a çevirmek için)
        self.target_id_map: Dict[str, int] = {}

        # =====================================================================
        # WEBSOCKET CLIENT
        # =====================================================================
        self.sio = socketio.Client(
            reconnection=self.auto_reconnect,
            reconnection_attempts=0,  # Sınırsız
            reconnection_delay=self.reconnect_delay,
            logger=False,
            engineio_logger=False
        )

        # WebSocket event handler'ları
        self._setup_socket_handlers()

        # =====================================================================
        # TIMER'LAR
        # =====================================================================
        # Periyodik yayın timer'ı
        timer_period = 1.0 / self.publish_rate
        self.publish_timer = self.create_timer(timer_period, self._publish_targets)

        # Eski hedefleri temizleme timer'ı
        self.cleanup_timer = self.create_timer(1.0, self._cleanup_stale_targets)

        # Bağlantı durumu kontrolü
        self.connection_timer = self.create_timer(5.0, self._check_connection)

        # =====================================================================
        # BAĞLANTIYI BAŞLAT
        # =====================================================================
        self._start_connection_thread()

        self.get_logger().info(f'🎯 MockTargetReceiver başlatıldı')
        self.get_logger().info(f'   Sunucu: {self.mock_server_url}')
        self.get_logger().info(f'   Yayın hızı: {self.publish_rate} Hz')

    def _setup_socket_handlers(self):
        """WebSocket event handler'larını ayarla"""

        @self.sio.event
        def connect():
            self.connected = True
            self.get_logger().info('🔗 Mock sunucuya bağlandı!')

        @self.sio.event
        def disconnect():
            self.connected = False
            self.get_logger().warn('🔌 Mock sunucu bağlantısı kesildi!')

        @self.sio.event
        def connect_error(data):
            self.connected = False
            self.get_logger().error(f'❌ Bağlantı hatası: {data}')

        @self.sio.on('telemetry')
        def on_telemetry(data):
            self._handle_telemetry(data)

        @self.sio.on('lock_response')
        def on_lock_response(data):
            self._handle_lock_response(data)

        @self.sio.on('lock_success')
        def on_lock_success(data):
            self._handle_lock_success(data)

    def _handle_telemetry(self, data: dict):
        """
        Mock sunucudan gelen telemetri verisini işle.
        """
        uav_id = data.get('id', 'unknown')

        # Kendi uçağımızı filtrele
        if uav_id == self.own_uav_id:
            return

        current_time = time.time()

        with self.targets_lock:
            if uav_id not in self.targets:
                self.targets[uav_id] = TargetState(id=uav_id)
                # ID mapping oluştur (string -> integer)
                self.target_id_map[uav_id] = len(self.target_id_map) + 1
                self.get_logger().info(f'🆕 Yeni hedef tespit edildi: {uav_id} (ID: {self.target_id_map[uav_id]})')

            target = self.targets[uav_id]
            target.lat = float(data.get('lat', 0.0))
            target.lon = float(data.get('lon', 0.0))
            target.alt = float(data.get('alt', 0.0))
            target.vx = float(data.get('vx', 0.0))
            target.vy = float(data.get('vy', 0.0))
            target.vz = float(data.get('vz', 0.0))
            target.yaw = float(data.get('yaw', 0.0))
            target.speed = float(data.get('speed', 0.0))
            target.timestamp = float(data.get('timestamp', current_time))
            target.status = data.get('status', 'ACTIVE')
            target.last_update = current_time

    def _handle_lock_response(self, data: dict):
        """Lock yanıtını logla"""
        self.get_logger().info(f'🔐 Lock Response: {data}')

    def _handle_lock_success(self, data: dict):
        """Lock başarı bilgisini logla"""
        self.get_logger().info(f'✅ Lock Success: {data}')

    def _get_target_heading(self, target: TargetState) -> float:
        """
        Hedefin heading açısını vx, vy'den hesapla.
        Returns: heading in degrees (0-360)
        """
        import math
        if abs(target.vx) < 0.01 and abs(target.vy) < 0.01:
            return target.yaw
        heading = math.degrees(math.atan2(target.vy, target.vx))
        if heading < 0:
            heading += 360.0
        return heading

    def _publish_targets(self):
        """
        Tüm hedefleri gps_tracking_node'un beklediği formatlarda yayınla:
        1. /competition/target_list (Float64MultiArray)
        2. /simulation/enemy_aircraft (JSON String)
        """
        if not self.connected:
            return

        with self.targets_lock:
            if not self.targets:
                return

            # ================================================================
            # 1. Float64MultiArray formatı (/competition/target_list)
            # Format: [id, lat, lon, alt, speed, heading] x N hedef
            # ================================================================
            target_array_msg = Float64MultiArray()
            target_data = []

            aircraft_list = []  # JSON için

            for uav_id, target in self.targets.items():
                # Integer ID al
                int_id = self.target_id_map.get(uav_id, 0)

                # Heading hesapla
                heading = self._get_target_heading(target)

                # Float64MultiArray için veri ekle
                target_data.extend([
                    float(int_id),
                    target.lat,
                    target.lon,
                    target.alt,
                    target.speed,
                    heading
                ])

                # JSON için aircraft objesi
                aircraft_list.append({
                    'id': int_id,
                    'lat': target.lat,
                    'lon': target.lon,
                    'alt': target.alt,
                    'speed': target.speed,
                    'heading': heading,
                    'vx': target.vx,
                    'vy': target.vy,
                    'vz': target.vz
                })

            # Float64MultiArray yayınla
            target_array_msg.data = target_data
            self.target_list_pub.publish(target_array_msg)

            # ================================================================
            # 2. JSON formatı (/simulation/enemy_aircraft)
            # ================================================================
            json_msg = String()
            json_msg.data = json.dumps({'aircraft': aircraft_list})
            self.enemy_aircraft_pub.publish(json_msg)

            # Hedef sayısı
            count_msg = Int32()
            count_msg.data = len(self.targets)
            self.target_count_pub.publish(count_msg)

    def _cleanup_stale_targets(self):
        """Zaman aşımına uğramış hedefleri temizle"""
        current_time = time.time()

        with self.targets_lock:
            stale_ids = []
            for uav_id, target in self.targets.items():
                if current_time - target.last_update > self.target_timeout:
                    stale_ids.append(uav_id)

            for uav_id in stale_ids:
                del self.targets[uav_id]
                if uav_id in self.target_id_map:
                    del self.target_id_map[uav_id]
                self.get_logger().warn(f'⏰ Hedef zaman aşımı: {uav_id}')

    def _check_connection(self):
        """Bağlantı durumunu kontrol et ve yeniden bağlan"""
        if not self.connected and self.auto_reconnect:
            self.get_logger().info('🔄 Yeniden bağlanmaya çalışılıyor...')
            self._start_connection_thread()

    def _start_connection_thread(self):
        """WebSocket bağlantısını ayrı thread'de başlat"""
        def connect_worker():
            try:
                if not self.sio.connected:
                    self.sio.connect(self.mock_server_url)
            except Exception as e:
                self.get_logger().error(f'Bağlantı hatası: {e}')
                self.connected = False

        thread = threading.Thread(target=connect_worker, daemon=True)
        thread.start()

    def send_lock_attempt(self, target_id: str, target_lat: float, target_lon: float):
        """
        Lock denemesi gönder.
        Bu fonksiyon dışarıdan çağrılabilir (örn: service callback).
        """
        if not self.connected:
            self.get_logger().warn('Lock gönderilemedi: Bağlantı yok')
            return False

        try:
            self.sio.emit('lock_attempt', {
                'id': target_id,
                'target_lat': target_lat,
                'target_lon': target_lon
            })
            self.get_logger().info(f'🔐 Lock denemesi gönderildi: {target_id}')
            return True
        except Exception as e:
            self.get_logger().error(f'Lock gönderme hatası: {e}')
            return False

    def destroy_node(self):
        """Node kapatılırken WebSocket bağlantısını kapat"""
        if self.sio.connected:
            self.sio.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = MockTargetReceiver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
