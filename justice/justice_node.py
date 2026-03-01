#!/usr/bin/env python3
"""
================================================================================
JUSTICE — GPS + GÖRSEL HİBRİT TAKİP NODE'U
================================================================================
L1 Adaptive Guidance (GPS) ve YOLO+TCTrack++ (Görsel) tabanlı
birleşik takip sistemi.

GPS takipte: L1 Guidance → Position Setpoint (NED pozisyon)
Görsel takipte: PID Servolama → Velocity Setpoint (NED hız)

Durum Makinesi:
  BEKLEME → KALKIS_BEKLE → OFFBOARD_GECIS → GPS_TAKIP ⟺ GORSEL_TAKIP → KILITLENME

GPS → Görsel geçiş koşulları:
  1. Kamera hedef tespit etti (target_area > 0)
  2. GPS mesafe < transition_distance (200m)
  3. Ardışık tespit >= transition_min_detections (5 kare)

Görsel → GPS dönüş koşulu:
  Hedef 10 saniye boyunca kaybolursa

Girdiler:
  /vision/target_state     (PointStamped)       — tracker_node'dan görsel hata
  PX4 topic'leri           (VehicleLocalPosition, VehicleGlobalPosition, vb.)
  /competition/target_list (Float64MultiArray)   — Hedef GPS konumları
  /simulation/enemy_aircraft (String)            — JSON hedef verisi

Çıktılar (PX4 micro-XRCE-DDS):
  /fmu/in/offboard_control_mode  (OffboardControlMode)
  /fmu/in/trajectory_setpoint    (TrajectorySetpoint)
  /fmu/in/vehicle_command        (VehicleCommand)

Yazar: HAVK Takımı — Justice Entegrasyonu
Tarih: 2026
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor

from typing import Dict, Optional
from collections import deque
import math
import json
import time

# ============================================================================
# L1 GUIDANCE MODÜLÜ (ROS2 Bağımsız — justice/l1_guidance.py)
# ============================================================================
try:
    from .l1_guidance import (
        L1Guidance,
        AircraftState,
        GuidanceCommand,
        TrackingState,
        L1GuidanceParams,
        TargetSelectorParams,
        StateMachineParams,
        geodetic_to_ned,
        calculate_distance_3d,
        calculate_bearing,
        normalize_angle,
    )
except (ImportError, ValueError):
    from l1_guidance import (
        L1Guidance,
        AircraftState,
        GuidanceCommand,
        TrackingState,
        L1GuidanceParams,
        TargetSelectorParams,
        StateMachineParams,
        geodetic_to_ned,
        calculate_distance_3d,
        calculate_bearing,
        normalize_angle,
    )

# ============================================================================
# PX4 MESAJ TİPLERİ
# ============================================================================
try:
    from px4_msgs.msg import (
        VehicleLocalPosition,
        VehicleGlobalPosition,
        VehicleAttitude,
        VehicleStatus,
        TrajectorySetpoint,
        OffboardControlMode,
        VehicleCommand,
    )
    PX4_MSGS_AVAILABLE = True
except ImportError:
    PX4_MSGS_AVAILABLE = False
    print("[UYARI] px4_msgs paketi bulunamadı!")

# Standart ROS2 mesajları
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PointStamped, PoseStamped


# ============================================================================
# PID KONTROLCÜ (Görsel Takip için)
# ============================================================================
class PIDController:
    """Basit PID Kontrolcü (Anti-Windup ile)."""

    def __init__(self, kp: float, ki: float, kd: float, output_limit: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.integral_limit = output_limit * 0.5

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0
        p = self.kp * error
        self.integral += error * dt
        self.integral = max(-self.integral_limit,
                            min(self.integral_limit, self.integral))
        i = self.ki * self.integral
        d = self.kd * (error - self.prev_error) / dt
        self.prev_error = error
        output = p + i + d
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update_gains(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd


# ============================================================================
# DURUM MAKİNESİ SABİTLERİ
# ============================================================================
class Durum:
    """Birleşik durum makinesi durumları"""
    BEKLEME        = 0   # PX4 bağlantısı bekleniyor
    KALKIS_BEKLE   = 1   # Manuel kalkış bekleniyor (ARM)
    OFFBOARD_GECIS = 2   # Offboard moduna geçiş
    GPS_TAKIP      = 3   # L1 Guidance ile GPS tabanlı takip (Position Setpoint)
    GORSEL_TAKIP   = 4   # PID ile görsel takip (Velocity Setpoint)
    KILITLENME     = 5   # Kilitlenme onaylandı (4s kuralı)


DURUM_ADLARI = {
    Durum.BEKLEME:        "BEKLEME",
    Durum.KALKIS_BEKLE:   "KALKIŞ_BEKLE",
    Durum.OFFBOARD_GECIS: "OFFBOARD_GEÇİŞ",
    Durum.GPS_TAKIP:      "GPS_TAKİP (L1)",
    Durum.GORSEL_TAKIP:   "GÖRSEL_TAKİP (PID)",
    Durum.KILITLENME:     "KİLİTLENME ✓",
}

# PX4 sabitleri
ARMING_STATE_ARMED = 2
NAVIGATION_STATE_OFFBOARD = 14


# ============================================================================
# ANA JUSTICE NODE
# ============================================================================
class JusticeNode(Node):
    """
    Justice — GPS + Görsel Hibrit Takip Node'u

    GPS_TAKIP:     L1 Guidance → Position Setpoint → PX4 kendi navigasyonu
    GORSEL_TAKIP:  PID Servolama → Velocity Setpoint → Doğrudan hız kontrolü

    Kontrol modu geçişi:
      GPS modunda:    OffboardControlMode.position = True
      Görsel modda:   OffboardControlMode.velocity = True
    """

    def __init__(self):
        super().__init__('justice_node')

        self.get_logger().info("=" * 60)
        self.get_logger().info("  JUSTICE — GPS + GÖRSEL HİBRİT TAKİP SİSTEMİ")
        self.get_logger().info("  L1 Adaptive Guidance + YOLO/TCTrack++ PID")
        self.get_logger().info("=" * 60)

        # ====================================================================
        # PARAMETRELERİ TANIMLA VE YÜKLE
        # ====================================================================
        self._declare_parameters()
        self._load_parameters()

        # ====================================================================
        # QoS PROFİLLERİ
        # ====================================================================
        self.px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.standard_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ====================================================================
        # DURUM DEĞİŞKENLERİ
        # ====================================================================
        # --- Durum makinesi ---
        self.durum = Durum.BEKLEME
        self.last_loop_time = time.time()
        self.offboard_setpoint_counter = 0
        self.offboard_retry_counter = 0
        self.bekleme_start_time = time.time()
        self.BEKLEME_TIMEOUT = 30.0

        # --- PX4 durumu ---
        self.px4_connected = False
        self.px4_armed = False
        self.px4_nav_state = 0
        self.vehicle_status_received = False
        self.last_vehicle_status_time = 0.0

        # --- Kendi uçak durumu (L1 Guidance için) ---
        self.own_state = AircraftState(id=0)
        self.current_heading = 0.0   # Radyan

        # --- Home pozisyonu ---
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_alt = 0.0
        self.home_set = False

        # --- Hedef listesi (L1 Guidance için — GPS hedefler) ---
        self.target_list: Dict[int, AircraftState] = {}

        # --- Görsel takip verileri ---
        self.vision_error_x = 0.0
        self.vision_error_y = 0.0
        self.vision_target_area = 0.0
        self.last_vision_time = 0.0
        self.consecutive_detections = 0

        # --- GPS → Hedefe mesafe (geçiş kararı için) ---
        self.gps_target_distance = float('inf')

        # --- Mevcut kontrol modu ---
        self._control_mode_position = True   # True = GPS/Position, False = Görsel/Velocity

        # --- Geçiş yumuşatması ---
        self._transition_in_progress = False
        self._transition_start_time = 0.0
        self._last_velocity_at_transition = [0.0, 0.0, 0.0]

        # --- Kilitlenme takibi (4s kuralı — görsel modda) ---
        self.visual_lock_start_time: Optional[float] = None
        self.visual_lock_confirmed = False

        # --- İstatistikler ---
        self.stats = {
            'successful_locks': 0,
            'gps_to_visual_transitions': 0,
            'visual_to_gps_fallbacks': 0,
            'state_transitions': 0,
        }

        # --- Debug sayaçları ---
        self._publish_counter = 0
        self._local_pos_received = False
        self._global_pos_received = False
        self._target_local_cb_count = 0
        self._target_global_cb_count = 0

        # ====================================================================
        # L1 GUIDANCE MODÜLÜ
        # ====================================================================
        self._init_l1_guidance()

        # ====================================================================
        # PID KONTROLCÜLERİ (Görsel Takip)
        # ====================================================================
        self.pid_yaw = PIDController(
            self.kp_yaw, self.ki_yaw, self.kd_yaw, self.max_yaw_rate)
        self.pid_pitch = PIDController(
            self.kp_pitch, self.ki_pitch, self.kd_pitch, self.max_vz)

        # ====================================================================
        # SUBSCRIBER'LAR
        # ====================================================================
        self._create_subscribers()

        # ====================================================================
        # PUBLISHER'LAR
        # ====================================================================
        self._create_publishers()

        # ====================================================================
        # TIMER'LAR
        # ====================================================================
        control_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(control_period, self._kontrol_dongusu)
        self.offboard_timer = self.create_timer(0.1, self._offboard_heartbeat)
        self.status_timer = self.create_timer(2.0, self._durum_logla)
        self.param_timer = self.create_timer(1.0, self._parametre_guncelle)
        self.connection_timer = self.create_timer(5.0, self._check_px4_connection)

        self.get_logger().info(f"Kontrol frekansı: {self.control_frequency} Hz")
        self.get_logger().info(f"L1 Mesafesi: {self.l1_distance} m")
        self.get_logger().info(f"GPS→Görsel geçiş: mesafe<{self.transition_distance}m, "
                               f"ardışık>={self.transition_min_detections}")
        self.get_logger().info(f"Görsel→GPS dönüş: {self.visual_lost_timeout}s timeout")
        self.get_logger().info("📡 PX4 bağlantısı bekleniyor...")

    # ========================================================================
    # PARAMETRE YÖNETİMİ
    # ========================================================================

    def _declare_parameters(self):
        """Tüm parametreleri tanımla — GPS + Görsel birleşik"""

        # --- L1 Guidance ---
        self.declare_parameter('l1_guidance.l1_distance', 50.0)
        self.declare_parameter('l1_guidance.l1_damping', 0.85)
        self.declare_parameter('l1_guidance.l1_period', 25.0)
        self.declare_parameter('l1_guidance.adaptive_l1', True)

        # --- Hedef Seçim Ağırlıkları ---
        self.declare_parameter('weights.distance', 0.25)
        self.declare_parameter('weights.angle', 0.30)
        self.declare_parameter('weights.speed', 0.15)
        self.declare_parameter('weights.camera_fov', 0.20)
        self.declare_parameter('weights.reachability', 0.10)

        # --- Açı Puanlama ---
        self.declare_parameter('angle_scoring.tail_angle_bonus', 2.5)
        self.declare_parameter('angle_scoring.head_on_penalty', 0.2)
        self.declare_parameter('angle_scoring.tail_cone_half_angle', 45.0)
        self.declare_parameter('angle_scoring.head_on_cone_half_angle', 30.0)

        # --- Uçak Limitleri ---
        self.declare_parameter('aircraft_limits.min_airspeed', 15.0)
        self.declare_parameter('aircraft_limits.max_airspeed', 35.0)
        self.declare_parameter('aircraft_limits.cruise_airspeed', 22.0)
        self.declare_parameter('aircraft_limits.max_bank_angle', 45.0)
        self.declare_parameter('aircraft_limits.loiter_radius', 40.0)

        # --- Smoothing ---
        self.declare_parameter('smoothing.heading_rate_limit', 15.0)
        self.declare_parameter('smoothing.altitude_rate_limit', 3.0)
        self.declare_parameter('smoothing.command_smoothing_alpha', 0.15)
        self.declare_parameter('smoothing.lpf_cutoff_heading', 0.5)

        # --- L1 Durum Makinesi ---
        self.declare_parameter('state_machine.lock_distance', 80.0)
        self.declare_parameter('state_machine.approach_distance', 150.0)
        self.declare_parameter('state_machine.loiter_trigger_distance', 25.0)
        self.declare_parameter('state_machine.lock_confirmation_time', 4.0)

        # --- Görsel Takip PID ---
        self.declare_parameter('visual.kp_yaw', 0.002)
        self.declare_parameter('visual.ki_yaw', 0.0001)
        self.declare_parameter('visual.kd_yaw', 0.001)
        self.declare_parameter('visual.kp_pitch', 0.003)
        self.declare_parameter('visual.ki_pitch', 0.0001)
        self.declare_parameter('visual.kd_pitch', 0.001)
        self.declare_parameter('visual.max_yaw_rate', 0.5)
        self.declare_parameter('visual.max_vz', 3.0)
        self.declare_parameter('visual.base_speed', 15.0)
        self.declare_parameter('visual.area_ref', 5000.0)
        self.declare_parameter('visual.kp_speed', 0.001)
        self.declare_parameter('visual.min_speed', 10.0)
        self.declare_parameter('visual.max_speed', 25.0)

        # --- Hibrit Geçiş ---
        self.declare_parameter('transition.gps_to_visual_distance', 200.0)
        self.declare_parameter('transition.min_consecutive_detections', 5)
        self.declare_parameter('transition.visual_lost_timeout', 10.0)

        # --- PX4 ---
        self.declare_parameter('px4.control_frequency', 50.0)
        self.declare_parameter('px4.namespace', '')
        self.declare_parameter('px4.mav_sys_id', 1)

        # --- Güvenlik ---
        self.declare_parameter('safety.min_altitude_agl', 50.0)

        # --- Namespace (Multi-vehicle) ---
        self.declare_parameter('namespaces.targets', '/px4_1,/px4_2')

    def _load_parameters(self):
        """Parametreleri oku"""
        # L1 Guidance
        self.l1_distance = self.get_parameter('l1_guidance.l1_distance').value
        self.l1_damping = self.get_parameter('l1_guidance.l1_damping').value
        self.l1_period = self.get_parameter('l1_guidance.l1_period').value
        self.adaptive_l1 = self.get_parameter('l1_guidance.adaptive_l1').value

        # Hedef seçim ağırlıkları
        self.w_distance = self.get_parameter('weights.distance').value
        self.w_angle = self.get_parameter('weights.angle').value
        self.w_speed = self.get_parameter('weights.speed').value
        self.w_camera_fov = self.get_parameter('weights.camera_fov').value
        self.w_reachability = self.get_parameter('weights.reachability').value

        # Açı puanlama
        self.tail_bonus = self.get_parameter('angle_scoring.tail_angle_bonus').value
        self.head_on_penalty = self.get_parameter('angle_scoring.head_on_penalty').value
        self.tail_cone = self.get_parameter('angle_scoring.tail_cone_half_angle').value
        self.head_on_cone = self.get_parameter('angle_scoring.head_on_cone_half_angle').value

        # Uçak limitleri
        self.min_airspeed = self.get_parameter('aircraft_limits.min_airspeed').value
        self.max_airspeed = self.get_parameter('aircraft_limits.max_airspeed').value
        self.cruise_airspeed = self.get_parameter('aircraft_limits.cruise_airspeed').value
        self.max_bank = self.get_parameter('aircraft_limits.max_bank_angle').value
        self.loiter_radius = self.get_parameter('aircraft_limits.loiter_radius').value

        # Smoothing
        self.heading_rate_limit = self.get_parameter('smoothing.heading_rate_limit').value
        self.altitude_rate_limit = self.get_parameter('smoothing.altitude_rate_limit').value
        self.smoothing_alpha = self.get_parameter('smoothing.command_smoothing_alpha').value
        self.lpf_cutoff = self.get_parameter('smoothing.lpf_cutoff_heading').value

        # Durum makinesi
        self.lock_distance = self.get_parameter('state_machine.lock_distance').value
        self.approach_distance = self.get_parameter('state_machine.approach_distance').value
        self.loiter_trigger = self.get_parameter('state_machine.loiter_trigger_distance').value
        self.lock_confirm_time = self.get_parameter('state_machine.lock_confirmation_time').value

        # Görsel PID
        self.kp_yaw = self.get_parameter('visual.kp_yaw').value
        self.ki_yaw = self.get_parameter('visual.ki_yaw').value
        self.kd_yaw = self.get_parameter('visual.kd_yaw').value
        self.kp_pitch = self.get_parameter('visual.kp_pitch').value
        self.ki_pitch = self.get_parameter('visual.ki_pitch').value
        self.kd_pitch = self.get_parameter('visual.kd_pitch').value
        self.max_yaw_rate = self.get_parameter('visual.max_yaw_rate').value
        self.max_vz = self.get_parameter('visual.max_vz').value
        self.base_speed = self.get_parameter('visual.base_speed').value
        self.area_ref = self.get_parameter('visual.area_ref').value
        self.kp_speed = self.get_parameter('visual.kp_speed').value
        self.min_speed = self.get_parameter('visual.min_speed').value
        self.max_speed = self.get_parameter('visual.max_speed').value

        # Hibrit geçiş
        self.transition_distance = self.get_parameter('transition.gps_to_visual_distance').value
        self.transition_min_detections = self.get_parameter('transition.min_consecutive_detections').value
        self.visual_lost_timeout = self.get_parameter('transition.visual_lost_timeout').value

        # PX4
        self.control_frequency = self.get_parameter('px4.control_frequency').value
        self.px4_ns = self.get_parameter('px4.namespace').value

        # Güvenlik
        self.min_alt = self.get_parameter('safety.min_altitude_agl').value

        # Namespace
        targets_str = self.get_parameter('namespaces.targets').value
        self.target_namespaces = [ns.strip() for ns in targets_str.split(',') if ns.strip()]

    def _parametre_guncelle(self):
        """Her 1 saniyede parametreleri yeniden oku ve PID güncelle."""
        old_kp_yaw = self.kp_yaw
        self._load_parameters()

        self.pid_yaw.update_gains(self.kp_yaw, self.ki_yaw, self.kd_yaw)
        self.pid_pitch.update_gains(self.kp_pitch, self.ki_pitch, self.kd_pitch)
        self.pid_yaw.output_limit = self.max_yaw_rate
        self.pid_pitch.output_limit = self.max_vz

    # ========================================================================
    # L1 GUIDANCE BAŞLATMA
    # ========================================================================

    def _init_l1_guidance(self):
        """L1 Guidance modülünü başlat"""
        l1_params = L1GuidanceParams(
            l1_distance=self.l1_distance,
            l1_damping=self.l1_damping,
            l1_period=self.l1_period,
            adaptive_l1=self.adaptive_l1,
            min_airspeed=self.min_airspeed,
            max_airspeed=self.max_airspeed,
            cruise_airspeed=self.cruise_airspeed,
            max_bank_angle=self.max_bank,
            loiter_radius=self.loiter_radius,
        )
        selector_params = TargetSelectorParams(
            w_distance=self.w_distance,
            w_angle=self.w_angle,
            w_speed=self.w_speed,
            w_camera_fov=self.w_camera_fov,
            w_reachability=self.w_reachability,
            tail_bonus=self.tail_bonus,
            head_on_penalty=self.head_on_penalty,
            tail_cone=self.tail_cone,
            head_on_cone=self.head_on_cone,
        )
        state_params = StateMachineParams(
            lock_distance=self.lock_distance,
            approach_distance=self.approach_distance,
            loiter_trigger=self.loiter_trigger,
            lock_confirm_time=self.lock_confirm_time,
        )
        self.l1_guidance = L1Guidance(
            l1_params=l1_params,
            selector_params=selector_params,
            state_params=state_params,
            sample_rate=self.control_frequency,
        )
        self.get_logger().info("L1 Guidance modülü başlatıldı")

    # ========================================================================
    # YARDIMCI: PX4 TOPIC İSİM OLUŞTURUCU
    # ========================================================================

    def _px4_topic(self, base: str) -> str:
        """Namespace-aware PX4 topic ismi oluştur"""
        if self.px4_ns:
            clean = self.px4_ns.lstrip('/')
            return f"/{clean}/{base}"
        return f"/{base}"

    # ========================================================================
    # SUBSCRIBER'LAR
    # ========================================================================

    def _create_subscribers(self):
        """Tüm subscriber'ları oluştur"""

        if PX4_MSGS_AVAILABLE:
            # --- Kendi uçak verileri (PX4) ---
            self.create_subscription(
                VehicleLocalPosition,
                self._px4_topic('fmu/out/vehicle_local_position'),
                self._own_local_pos_cb, self.px4_qos)
            self.create_subscription(
                VehicleLocalPosition,
                self._px4_topic('fmu/out/vehicle_local_position_v1'),
                self._own_local_pos_cb, self.px4_qos)
            self.create_subscription(
                VehicleGlobalPosition,
                self._px4_topic('fmu/out/vehicle_global_position'),
                self._own_global_pos_cb, self.px4_qos)
            self.create_subscription(
                VehicleAttitude,
                self._px4_topic('fmu/out/vehicle_attitude'),
                self._own_attitude_cb, self.px4_qos)
            # VehicleStatus — BEST_EFFORT/VOLATILE (standart PX4 QoS)
            self.create_subscription(
                VehicleStatus,
                self._px4_topic('fmu/out/vehicle_status'),
                self._vehicle_status_cb, self.px4_qos)
            self.create_subscription(
                VehicleStatus,
                self._px4_topic('fmu/out/vehicle_status_v1'),
                self._vehicle_status_cb, self.px4_qos)
                
            # VehicleStatus — RELIABLE/TRANSIENT_LOCAL (bazı PX4 build'larında)
            _status_qos_tl = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.create_subscription(
                VehicleStatus,
                self._px4_topic('fmu/out/vehicle_status'),
                self._vehicle_status_cb, _status_qos_tl)
            self.create_subscription(
                VehicleStatus,
                self._px4_topic('fmu/out/vehicle_status_v1'),
                self._vehicle_status_cb, _status_qos_tl)

        # --- Görsel takip verileri (tracker_node'dan) ---
        self.create_subscription(
            PointStamped,
            '/vision/target_state',
            self._vision_cb, 10)

        # --- GPS hedef listesi (yarışma sunucusu / mock_target_receiver) ---
        self.create_subscription(
            Float64MultiArray,
            '/competition/target_list',
            self._target_list_cb, self.standard_qos)
        self.create_subscription(
            String,
            '/simulation/enemy_aircraft',
            self._enemy_aircraft_cb, self.standard_qos)

        # --- PX4 SITL Multi-Vehicle hedef subscriber'ları ---
        self._target_subs = []
        if PX4_MSGS_AVAILABLE and self.target_namespaces:
            for idx, t_ns in enumerate(self.target_namespaces):
                tid = idx + 1

                def _make_topic(base, ns):
                    ns = ns.rstrip('/')
                    return f"{ns}/{base}" if ns else f"/{base}"

                def _mk_local_cb(t_id, t_namespace):
                    def cb(msg):
                        self._target_local_cb(msg, t_id, t_namespace)
                    return cb

                def _mk_global_cb(t_id, t_namespace):
                    def cb(msg):
                        self._target_global_cb(msg, t_id, t_namespace)
                    return cb

                for suffix in ['', '_v1']:
                    self._target_subs.append(self.create_subscription(
                        VehicleLocalPosition,
                        _make_topic(f'fmu/out/vehicle_local_position{suffix}', t_ns),
                        _mk_local_cb(tid, t_ns), self.px4_qos))
                    self._target_subs.append(self.create_subscription(
                        VehicleGlobalPosition,
                        _make_topic(f'fmu/out/vehicle_global_position{suffix}', t_ns),
                        _mk_global_cb(tid, t_ns), self.px4_qos))

                self.get_logger().info(f"🎯 Hedef {tid} subscriber: namespace='{t_ns}'")

        self.get_logger().info("Subscriber'lar oluşturuldu")

    # ========================================================================
    # PUBLISHER'LAR
    # ========================================================================

    def _create_publishers(self):
        """Tüm publisher'ları oluştur"""
        if PX4_MSGS_AVAILABLE:
            self.trajectory_pub = self.create_publisher(
                TrajectorySetpoint,
                self._px4_topic('fmu/in/trajectory_setpoint'), self.px4_qos)
            self.offboard_mode_pub = self.create_publisher(
                OffboardControlMode,
                self._px4_topic('fmu/in/offboard_control_mode'), self.px4_qos)
            self.command_pub = self.create_publisher(
                VehicleCommand,
                self._px4_topic('fmu/in/vehicle_command'), self.px4_qos)

        # Debug / Görselleştirme
        self.debug_pub = self.create_publisher(String, '/justice/debug_info', self.standard_qos)
        self.state_pub = self.create_publisher(String, '/justice/state', self.standard_qos)
        self.virtual_target_pub = self.create_publisher(PoseStamped, '/justice/virtual_target', self.standard_qos)
        self.target_scores_pub = self.create_publisher(String, '/justice/target_scores', self.standard_qos)

        self.get_logger().info("Publisher'lar oluşturuldu")

    # ========================================================================
    # CALLBACK: KENDİ UÇAK VERİLERİ
    # ========================================================================

    def _own_local_pos_cb(self, msg):
        """PX4 local position (NED)"""
        self.own_state.x = msg.x
        self.own_state.y = msg.y
        self.own_state.z = msg.z
        self.own_state.vx = msg.vx
        self.own_state.vy = msg.vy
        self.own_state.vz = msg.vz
        self.own_state.heading = math.degrees(msg.heading)
        self.own_state.ground_speed = math.sqrt(msg.vx**2 + msg.vy**2)
        self.current_heading = msg.heading   # Radyan (body→NED dönüşümü için)

        # Home pozisyonunu ayarla
        if not self.home_set:
            if hasattr(msg, 'xy_global') and msg.xy_global:
                if hasattr(msg, 'ref_lat') and msg.ref_lat != 0.0:
                    self.home_lat = msg.ref_lat
                    self.home_lon = msg.ref_lon
                    self.home_alt = msg.ref_alt if hasattr(msg, 'ref_alt') else 0.0
                    self.home_set = True
                    self.get_logger().info(
                        f"✅ Home ayarlandı (ref): {self.home_lat:.6f}, {self.home_lon:.6f}")

        if not self._local_pos_received:
            self._local_pos_received = True
            self.get_logger().info(f"📍 Local position: x={msg.x:.1f}, y={msg.y:.1f}, z={msg.z:.1f}")

    def _own_global_pos_cb(self, msg):
        """PX4 global GPS position"""
        self.own_state.latitude = msg.lat
        self.own_state.longitude = msg.lon
        self.own_state.altitude = msg.alt
        self.own_state.timestamp = self.get_clock().now().nanoseconds / 1e9

        if not self.home_set and msg.lat != 0.0 and msg.lon != 0.0:
            self.home_lat = msg.lat
            self.home_lon = msg.lon
            self.home_alt = msg.alt
            self.home_set = True
            self.get_logger().info(
                f"✅ Home ayarlandı (GPS): {self.home_lat:.6f}, {self.home_lon:.6f}")

        if not self._global_pos_received:
            self._global_pos_received = True
            self.get_logger().info(f"🌍 Global position: lat={msg.lat:.6f}, lon={msg.lon:.6f}")

    def _own_attitude_cb(self, msg):
        """Quaternion → Euler roll/pitch"""
        q = msg.q
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1]**2 + q[2]**2)
        self.own_state.roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        if abs(sinp) >= 1:
            self.own_state.pitch = math.degrees(math.copysign(math.pi / 2, sinp))
        else:
            self.own_state.pitch = math.degrees(math.asin(sinp))

    def _vehicle_status_cb(self, msg):
        """PX4 durum bilgisi (ARM, nav_state)"""
        was_connected = self.px4_connected
        self.px4_connected = True
        self.vehicle_status_received = True
        self.px4_armed = (msg.arming_state == ARMING_STATE_ARMED)
        self.px4_nav_state = msg.nav_state
        self.last_vehicle_status_time = time.time()
        if not was_connected:
            self.get_logger().info("✅ PX4 BAĞLANDI!")

    # ========================================================================
    # CALLBACK: GÖRSEL TAKİP (tracker_node)
    # ========================================================================

    def _vision_cb(self, msg):
        """
        tracker_node'dan gelen hedef verisi.
        x = yatay piksel hatası, y = dikey piksel hatası, z = bbox alanı (0=hedef yok)
        """
        self.vision_error_x = msg.point.x
        self.vision_error_y = msg.point.y
        self.vision_target_area = msg.point.z

        if self.vision_target_area > 0.0:
            self.last_vision_time = time.time()
            self.consecutive_detections += 1
        else:
            self.consecutive_detections = 0

    # ========================================================================
    # CALLBACK: GPS HEDEF VERİLERİ
    # ========================================================================

    def _target_list_cb(self, msg):
        """Yarışma sunucusu / mock_target_receiver'dan hedef listesi"""
        data = msg.data
        num_targets = len(data) // 6
        current_time = self.get_clock().now().nanoseconds / 1e9

        for i in range(num_targets):
            idx = i * 6
            target_id = int(data[idx])
            if target_id not in self.target_list:
                self.target_list[target_id] = AircraftState(id=target_id)
            t = self.target_list[target_id]
            t.latitude = data[idx + 1]
            t.longitude = data[idx + 2]
            t.altitude = data[idx + 3]
            t.ground_speed = data[idx + 4]
            t.heading = data[idx + 5]
            t.timestamp = current_time
            if self.home_set:
                n, e, d = geodetic_to_ned(t.latitude, t.longitude, t.altitude,
                                          self.home_lat, self.home_lon, self.home_alt)
                t.x, t.y, t.z = n, e, d
                heading_rad = math.radians(t.heading)
                t.vx = t.ground_speed * math.cos(heading_rad)
                t.vy = t.ground_speed * math.sin(heading_rad)
                t.vz = 0.0

    def _enemy_aircraft_cb(self, msg):
        """Simülasyondan JSON hedef verisi"""
        try:
            data = json.loads(msg.data)
            current_time = self.get_clock().now().nanoseconds / 1e9
            for aircraft in data.get('aircraft', []):
                tid = aircraft.get('id', 0)
                if tid not in self.target_list:
                    self.target_list[tid] = AircraftState(id=tid)
                t = self.target_list[tid]
                t.latitude = aircraft.get('lat', 0.0)
                t.longitude = aircraft.get('lon', 0.0)
                t.altitude = aircraft.get('alt', 0.0)
                t.ground_speed = aircraft.get('speed', 0.0)
                t.heading = aircraft.get('heading', 0.0)
                t.timestamp = current_time
                if self.home_set:
                    n, e, d = geodetic_to_ned(t.latitude, t.longitude, t.altitude,
                                              self.home_lat, self.home_lon, self.home_alt)
                    t.x, t.y, t.z = n, e, d
                    heading_rad = math.radians(t.heading)
                    t.vx = t.ground_speed * math.cos(heading_rad)
                    t.vy = t.ground_speed * math.sin(heading_rad)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"JSON parse hatası: {e}")

    def _target_local_cb(self, msg, target_id: int, target_ns: str):
        """PX4 SITL multi-vehicle: Hedef velocity (frame-bağımsız)"""
        self._target_local_cb_count += 1
        if target_id not in self.target_list:
            self.target_list[target_id] = AircraftState(id=target_id)
        t = self.target_list[target_id]
        t.vx = msg.vx
        t.vy = msg.vy
        t.vz = msg.vz
        t.heading = math.degrees(msg.heading)
        t.ground_speed = math.sqrt(msg.vx**2 + msg.vy**2)
        t.timestamp = self.get_clock().now().nanoseconds / 1e9

    def _target_global_cb(self, msg, target_id: int, target_ns: str):
        """PX4 SITL multi-vehicle: Hedef GPS → NED dönüşüm"""
        self._target_global_cb_count += 1
        if target_id not in self.target_list:
            self.target_list[target_id] = AircraftState(id=target_id)
        t = self.target_list[target_id]
        t.latitude = msg.lat
        t.longitude = msg.lon
        t.altitude = msg.alt
        t.timestamp = self.get_clock().now().nanoseconds / 1e9
        if self.home_set:
            n, e, d = geodetic_to_ned(t.latitude, t.longitude, t.altitude,
                                      self.home_lat, self.home_lon, self.home_alt)
            t.x, t.y, t.z = n, e, d

    # ========================================================================
    # ANA KONTROL DÖNGÜSÜ (50 Hz)
    # ========================================================================

    def _kontrol_dongusu(self):
        """Ana kontrol döngüsü — Durum makinesini çalıştırır"""
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now

        # Offboard heartbeat her zaman gönderilir (ayrı timer'da)
        # Varsayılan setpoint (offboard kalması için)
        if self.durum >= Durum.OFFBOARD_GECIS:
            self._publish_velocity_setpoint(self.base_speed, 0.0, 0.0, 0.0)

        # --- Eski hedefleri temizle ---
        self._cleanup_stale_targets(now)

        # --- Hedefe mesafe güncelle (geçiş kararı için) ---
        self._update_target_distance()

        # --- DURUM MAKİNESİ ---
        if self.durum == Durum.BEKLEME:
            self._durum_bekleme()
        elif self.durum == Durum.KALKIS_BEKLE:
            self._durum_kalkis_bekle()
        elif self.durum == Durum.OFFBOARD_GECIS:
            self._durum_offboard_gecis()
        elif self.durum == Durum.GPS_TAKIP:
            self._durum_gps_takip(dt)
        elif self.durum == Durum.GORSEL_TAKIP:
            self._durum_gorsel_takip(dt)
        elif self.durum == Durum.KILITLENME:
            self._durum_kilitlenme(dt)

    # ========================================================================
    # DURUM: BEKLEME — PX4 Bağlantısı Bekleniyor
    # ========================================================================

    def _durum_bekleme(self):
        if self.px4_connected:
            self._change_durum(Durum.KALKIS_BEKLE, "PX4 bağlandı!")
            return
        elapsed = time.time() - self.bekleme_start_time
        if elapsed > self.BEKLEME_TIMEOUT:
            self.get_logger().warn(f"⚠️ {self.BEKLEME_TIMEOUT:.0f}s VehicleStatus yok, devam ediliyor")
            self._change_durum(Durum.OFFBOARD_GECIS, "Timeout, VehicleStatus olmadan devam")
            self.offboard_setpoint_counter = 0

    # ========================================================================
    # DURUM: KALKIŞ_BEKLE — ARM Bekleniyor
    # ========================================================================

    def _durum_kalkis_bekle(self):
        if self.px4_armed:
            self._change_durum(Durum.OFFBOARD_GECIS, "Uçak ARMED!")
            self.offboard_setpoint_counter = 0
            self.offboard_retry_counter = 0

    # ========================================================================
    # DURUM: OFFBOARD_GECIS — Offboard Moduna Geçiş
    # ========================================================================

    def _durum_offboard_gecis(self):
        self.offboard_setpoint_counter += 1

        if self.offboard_setpoint_counter < 50:
            return

        # VehicleStatus'tan nav_state onayı alındıysa GPS takibine geç
        if self.px4_nav_state == NAVIGATION_STATE_OFFBOARD:
            self._change_durum(Durum.GPS_TAKIP, "Offboard BAŞARILI!")
            return

        if self.offboard_setpoint_counter % 50 == 0:
            self.offboard_retry_counter += 1
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0, param2=6.0)
            self.get_logger().info(
                f"⏳ Offboard deneme #{self.offboard_retry_counter} "
                f"(nav_state={self.px4_nav_state}, status_alindi={self.vehicle_status_received})")

        # Timeout: 20 denemeden sonra VehicleStatus olmadan devam et
        # (PX4 offboard'da ama biz göremiyorsak bile sistemi bloke etme)
        if self.offboard_retry_counter >= 20:
            self.get_logger().warn(
                "⚠️ VehicleStatus alınamadı ama Offboard/ARM komutları gönderildi. "
                "GPS takibine geçiliyor (nav_state doğrulamasız).")
            self._change_durum(Durum.GPS_TAKIP, "Offboard timeout — VehicleStatus yok")

    # ========================================================================
    # DURUM: GPS_TAKIP — L1 Guidance ile Position Setpoint
    # ========================================================================

    # Hız bazlı yaklaşma için mesafe eşiği (bu mesafenin üstünde velocity modu)
    VELOCITY_CHASE_DISTANCE = 80.0  # metre

    def _durum_gps_takip(self, dt: float):
        """
        GPS Takip: Hedefe uzakken velocity modunda hızlı kovalama,
        yaklaştığında L1 position modunda hassas navigasyon.
        """
        # Offboard kontrolü
        if self.px4_nav_state != NAVIGATION_STATE_OFFBOARD:
            self.get_logger().warn("⚠️ Offboard modundan çıkıldı!")
            self._change_durum(Durum.OFFBOARD_GECIS, "Offboard kayboldu")
            self.offboard_setpoint_counter = 0
            return

        # Home set olmadan L1 hesaplaması yapılamaz
        if not self.home_set:
            self._publish_safe_loiter()
            return

        # --- L1 GUIDANCE GÜNCELLE ---
        current_time = self.get_clock().now().nanoseconds / 1e9
        command = self.l1_guidance.update(
            own_state=self.own_state,
            targets=self.target_list,
            current_time=current_time,
        )

        # --- YAKINLIK BAZLI MOD SEÇİMİ ---
        if command.is_valid:
            # ===== POSITION MODU — L1 Hassas Navigasyon =====
            # Gerekli mesafe limiti ne olursa olsun, sabun kanatlı araçlarda düzgün yönelim için
            # her zaman Position setpoint kullanılır. Hız ayarı (kovalamaca) L1 Guidance tarafından hesaplanır.
            self._control_mode_position = True
            self._publish_position_setpoint(command)
            
            # (Opsiyonel: Eğer L1 tarafından istenen hızın PX4'te uygulanması için komut yollanması gerekiyorsa,
            # DO_CHANGE_SPEED çağrısı burada yapılabilir.)
            
            self._publish_virtual_target()
        else:
            self._control_mode_position = True
            self._publish_safe_loiter()

        # --- L1 durumu güncelle (debug) ---
        l1_state = self.l1_guidance.get_state()
        self._publish_target_scores()

        # --- HİBRİT GEÇİŞ KONTROLÜ: GPS → GÖRSEL ---
        now = time.time()
        vision_active = (self.vision_target_area > 0.0 and
                         (now - self.last_vision_time) < 2.0)
        distance_ok = self.gps_target_distance < self.transition_distance
        detections_ok = self.consecutive_detections >= self.transition_min_detections

        if vision_active and distance_ok and detections_ok:
            # Geçiş anında mevcut hızı kaydet (yumuşak geçiş için)
            self._last_velocity_at_transition = [
                self.own_state.vx, self.own_state.vy, self.own_state.vz]
            self._control_mode_position = False
            self.pid_yaw.reset()
            self.pid_pitch.reset()
            self.visual_lock_start_time = None
            self.visual_lock_confirmed = False
            self.stats['gps_to_visual_transitions'] += 1
            self._change_durum(Durum.GORSEL_TAKIP,
                f"Hedef kamerada! mesafe={self.gps_target_distance:.0f}m, "
                f"ardışık={self.consecutive_detections}")

    def _chase_target_velocity(self, command: GuidanceCommand):
        """
        Hedefe uzakken velocity modunda yüksek hızla kovalama.
        Hedef yönünde max_airspeed ile uçar + irtifa eşitler.
        """
        # Hedef yönüne olan birim vektör
        dn = command.position_north - self.own_state.x
        de = command.position_east - self.own_state.y
        dd = command.position_down - self.own_state.z
        dist_2d = math.sqrt(dn**2 + de**2)

        # Hız büyüklüğü: Mesafeye göre hızlan
        speed_cmd = getattr(command, 'airspeed_setpoint', self.max_airspeed)
        speed_cmd = max(speed_cmd, self.max_airspeed)  # Uzaktayken daima max hız

        if dist_2d > 1.0:
            vn = speed_cmd * (dn / dist_2d)
            ve = speed_cmd * (de / dist_2d)
        else:
            heading = self.current_heading
            vn = speed_cmd * math.cos(heading)
            ve = speed_cmd * math.sin(heading)

        # Dikey hız: Hedefe irtifa eşitleme (yumuşak)
        vz = max(-3.0, min(3.0, dd * 0.5))  # Hedefe doğru dikey hız, max ±3 m/s

        # NED velocity olarak gönder (body dönüşümü YOK — direkt NED)
        self._publish_ned_velocity(vn, ve, vz)

        # Debug log (1 Hz)
        self._publish_counter += 1
        if self._publish_counter % int(self.control_frequency) == 0:
            own_speed = math.sqrt(self.own_state.vx**2 + self.own_state.vy**2)
            self.get_logger().info(
                f"[GPS-CHASE] VelCmd={speed_cmd:.1f}m/s | "
                f"ActualSpd={own_speed:.1f}m/s | "
                f"L1_State={self.l1_guidance.get_state().name} | "
                f"Dist={self.gps_target_distance:.1f}m",
                throttle_duration_sec=1.0)

    def _publish_ned_velocity(self, vn: float, ve: float, vd: float):
        """Direkt NED velocity setpoint gönder (body dönüşümü olmadan)."""
        if not PX4_MSGS_AVAILABLE:
            return
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float('nan')] * 3
        msg.velocity = [float(vn), float(ve), float(vd)]
        msg.acceleration = [float('nan')] * 3
        msg.jerk = [float('nan')] * 3
        msg.yaw = float('nan')
        msg.yawspeed = float('nan')
        self.trajectory_pub.publish(msg)

    # ========================================================================
    # DURUM: GORSEL_TAKIP — PID ile Velocity Setpoint
    # ========================================================================

    def _durum_gorsel_takip(self, dt: float):
        """
        Görsel takip: PID kontrolcülerle yaw rate + dikey hız + ileri hız.
        Velocity Setpoint modunda çalışır (body → NED dönüşümü yapılır).
        Hedef 10 saniye kaybolursa GPS_TAKIP'e geri döner.
        """
        # Offboard kontrolü
        if self.px4_nav_state != NAVIGATION_STATE_OFFBOARD:
            self.get_logger().warn("⚠️ Offboard modundan çıkıldı!")
            self._change_durum(Durum.OFFBOARD_GECIS, "Offboard kayboldu")
            self.offboard_setpoint_counter = 0
            return

        now = time.time()
        time_since_vision = now - self.last_vision_time

        # --- HEDEF KAYIP KONTROLÜ → 10s timeout → GPS'e dön ---
        if time_since_vision > self.visual_lost_timeout or self.vision_target_area <= 0.0:
            if time_since_vision > self.visual_lost_timeout:
                self._control_mode_position = True
                self.pid_yaw.reset()
                self.pid_pitch.reset()
                self.consecutive_detections = 0
                self.visual_lock_start_time = None
                self.visual_lock_confirmed = False
                self.stats['visual_to_gps_fallbacks'] += 1
                self._change_durum(Durum.GPS_TAKIP,
                    f"Hedef kayıp {time_since_vision:.1f}s — GPS'e dönülüyor")
                return

        # --- PID HESAPLAMA (hedef görünüyorsa) ---
        if self.vision_target_area > 0.0 and time_since_vision < 2.0:
            # 1. YAW KONTROLÜ
            yawspeed = self.pid_yaw.compute(self.vision_error_x, dt)

            # 2. PITCH KONTROLÜ (NED: vz negatif = yukarı)
            vz = -self.pid_pitch.compute(self.vision_error_y, dt)

            # 3. HIZ KONTROLÜ (hedefe olan alana göre)
            area_error = self.area_ref - self.vision_target_area
            speed_adj = area_error * self.kp_speed
            vx = self.base_speed + speed_adj
            vx = max(self.min_speed, min(self.max_speed, vx))

            # Velocity setpoint gönder
            self._publish_velocity_setpoint(vx, 0.0, vz, yawspeed)

            # --- 4 SANİYE KİLİTLENME KONTROLÜ ---
            # (tracker_node zaten kendi içinde kontrol ediyor ama burada
            #  flight controller seviyesinde de takip edelim)
            self._check_visual_lock(now)
        else:
            # Hedef kısa süredir kayıp — düz uçuşa devam et
            self._publish_velocity_setpoint(self.base_speed, 0.0, 0.0, 0.0)

    # ========================================================================
    # DURUM: KİLİTLENME — 4s Kuralı Onaylandı
    # ========================================================================

    def _durum_kilitlenme(self, dt: float):
        """Kilitlenme durumu — görsel takibe devam, hedef kaybolursa GPS'e dön"""
        now = time.time()
        time_since_vision = now - self.last_vision_time

        if time_since_vision > self.visual_lost_timeout:
            self._control_mode_position = True
            self.pid_yaw.reset()
            self.pid_pitch.reset()
            self.consecutive_detections = 0
            self.visual_lock_start_time = None
            self.visual_lock_confirmed = False
            self.stats['visual_to_gps_fallbacks'] += 1
            self._change_durum(Durum.GPS_TAKIP,
                f"Kilitlenme sonrası hedef kayıp — GPS'e dönülüyor")
            return

        # Görsel takip devam ediyor
        if self.vision_target_area > 0.0 and time_since_vision < 2.0:
            yawspeed = self.pid_yaw.compute(self.vision_error_x, dt)
            vz = -self.pid_pitch.compute(self.vision_error_y, dt)
            area_error = self.area_ref - self.vision_target_area
            speed_adj = area_error * self.kp_speed
            vx = self.base_speed + speed_adj
            vx = max(self.min_speed, min(self.max_speed, vx))
            self._publish_velocity_setpoint(vx, 0.0, vz, yawspeed)
        else:
            self._publish_velocity_setpoint(self.base_speed, 0.0, 0.0, 0.0)

    # ========================================================================
    # GÖRSEL KİLİTLENME KONTROLÜ (4 SANİYE KURALI)
    # ========================================================================

    def _check_visual_lock(self, now: float):
        """
        Görsel takipte kilitlenme kontrolü.
        Hedef yeterince büyük ve merkezde ise 4 saniye sayacı başlatır.
        """
        # Basit kontrol: hedef alanı yeterince büyükse ve aktifse
        if self.vision_target_area > self.area_ref * 0.3:
            if self.visual_lock_start_time is None:
                self.visual_lock_start_time = now

            lock_duration = now - self.visual_lock_start_time
            if lock_duration >= self.lock_confirm_time and not self.visual_lock_confirmed:
                self.visual_lock_confirmed = True
                self.stats['successful_locks'] += 1
                self.get_logger().info(
                    f"🔒 GÖRSEL KİLİTLENME ONAYLANDI! Süre: {lock_duration:.2f}s")
                self._change_durum(Durum.KILITLENME, "4s kilitlenme başarılı!")
        else:
            # Hedef yeterince büyük/merkezi değil — sayacı sıfırla
            self.visual_lock_start_time = None

    # ========================================================================
    # PX4 MESAJ GÖNDERİCİLERİ
    # ========================================================================

    def _offboard_heartbeat(self):
        """
        Offboard heartbeat — sürekli gönderilmeli.
        Kontrol moduna göre position veya velocity flag'i değişir.
        """
        if not PX4_MSGS_AVAILABLE:
            return

        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        if self._control_mode_position:
            # GPS L1 Guidance: Position mod
            msg.position = True
            msg.velocity = False
        else:
            # Görsel PID: Velocity mod
            msg.position = False
            msg.velocity = True

        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_mode_pub.publish(msg)

        # İlk 150 setpoint sonra offboard moda geç
        self.offboard_setpoint_counter += 1
        if self.offboard_setpoint_counter == 150:
            self.get_logger().info("🚀 Offboard mod geçişi başlatılıyor...")
            self._engage_offboard_mode()

    def _publish_position_setpoint(self, command: GuidanceCommand):
        """
        L1 Guidance → PX4'e Position Setpoint gönder (yakın mesafe).
        PX4 kendi TECS/L1 navigasyonuyla bu noktaya uçar.
        """
        if not PX4_MSGS_AVAILABLE:
            return

        self._publish_counter += 1
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Position (NED) — Hedefin gerçek irtifasını kullan
        if (not math.isnan(command.position_north) and
            not math.isnan(command.position_east) and
            not math.isnan(command.position_down)):
            msg.position[0] = command.position_north
            msg.position[1] = command.position_east
            msg.position[2] = command.position_down
        else:
            msg.position = [float('nan')] * 3

        # Velocity & yaw → NaN (PX4 hesaplasın)
        msg.velocity = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.jerk = [float('nan')] * 3
        msg.yaw = float('nan')
        msg.yawspeed = float('nan')

        self.trajectory_pub.publish(msg)

        # Debug log (1 Hz)
        if self._publish_counter % int(self.control_frequency) == 0:
            self.get_logger().info(
                f"[GPS-L1] PosSet=[N={msg.position[0]:.1f} E={msg.position[1]:.1f} "
                f"D={msg.position[2]:.1f}] | "
                f"L1_State={self.l1_guidance.get_state().name} | "
                f"Dist={self.gps_target_distance:.1f}m",
                throttle_duration_sec=1.0)

    def _publish_velocity_setpoint(self, vx_body: float, vy_body: float,
                                    vz: float, yawspeed: float):
        """
        Görsel PID → PX4'e Velocity Setpoint gönder.
        Body frame → NED frame dönüşümü yapılır.
        """
        if not PX4_MSGS_AVAILABLE:
            return

        # Body → NED dönüşümü
        heading = self.current_heading
        vx_ned = vx_body * math.cos(heading) - vy_body * math.sin(heading)
        vy_ned = vx_body * math.sin(heading) + vy_body * math.cos(heading)

        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float('nan')] * 3
        msg.velocity = [float(vx_ned), float(vy_ned), float(vz)]
        msg.acceleration = [float('nan')] * 3
        msg.jerk = [float('nan')] * 3
        msg.yaw = float('nan')
        msg.yawspeed = float(yawspeed)

        self.trajectory_pub.publish(msg)

    def _publish_safe_loiter(self):
        """Hedef yokken güvenli düz uçuş komutu"""
        if not PX4_MSGS_AVAILABLE:
            return

        heading_rad = math.radians(self.own_state.heading) if self.own_state.heading != 0.0 else 0.0

        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float('nan')] * 3
        msg.velocity = [
            self.cruise_airspeed * math.cos(heading_rad),
            self.cruise_airspeed * math.sin(heading_rad),
            0.0,
        ]
        msg.yaw = heading_rad
        msg.yawspeed = float('nan')
        msg.acceleration = [float('nan')] * 3
        msg.jerk = [float('nan')] * 3
        self.trajectory_pub.publish(msg)

    def _engage_offboard_mode(self):
        """PX4'ü offboard moduna geçir ve ARM et"""
        if not PX4_MSGS_AVAILABLE:
            return
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0, param2=6.0)
        self.get_logger().info("📡 Offboard mod komutu gönderildi")

        if not self.px4_armed:
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0)
            self.get_logger().info("🔓 ARM komutu gönderildi")

    def _send_vehicle_command(self, command: int, param1=0.0, param2=0.0,
                               param3=0.0, param7=0.0):
        """PX4'e VehicleCommand gönder"""
        if not PX4_MSGS_AVAILABLE:
            return
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param7 = param7
        sys_id = self.get_parameter('px4.mav_sys_id').value
        msg.target_system = sys_id
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    # ========================================================================
    # YARDIMCI FONKSİYONLAR
    # ========================================================================

    def _change_durum(self, new_durum: int, reason: str = ""):
        """Durum değiştir ve logla"""
        old_name = DURUM_ADLARI.get(self.durum, "?")
        new_name = DURUM_ADLARI.get(new_durum, "?")
        self.durum = new_durum
        self.stats['state_transitions'] += 1

        emoji = "🔄"
        if new_durum == Durum.GPS_TAKIP:
            emoji = "📡"
        elif new_durum == Durum.GORSEL_TAKIP:
            emoji = "🎯"
        elif new_durum == Durum.KILITLENME:
            emoji = "🔒"

        self.get_logger().info(f"{emoji} DURUM: {old_name} → {new_name} ({reason})")

        # ROS2 state topic'e yayınla
        state_msg = String()
        state_msg.data = new_name
        self.state_pub.publish(state_msg)

    def _update_target_distance(self):
        """L1'in kilitli hedefine mesafeyi güncelle"""
        locked_id = self.l1_guidance.get_locked_target_id()
        if locked_id >= 0 and locked_id in self.target_list:
            own_pos = self.own_state.get_position_ned()
            target_pos = self.target_list[locked_id].get_position_ned()
            self.gps_target_distance = calculate_distance_3d(own_pos, target_pos)
        elif self.target_list:
            # Kilitli hedef yoksa en yakın hedefi kullan
            own_pos = self.own_state.get_position_ned()
            min_dist = float('inf')
            for t in self.target_list.values():
                d = calculate_distance_3d(own_pos, t.get_position_ned())
                min_dist = min(min_dist, d)
            self.gps_target_distance = min_dist
        else:
            self.gps_target_distance = float('inf')

    def _cleanup_stale_targets(self, current_time: float):
        """Timeout olan hedefleri temizle (5s)"""
        ros_time = self.get_clock().now().nanoseconds / 1e9
        stale = [tid for tid, t in self.target_list.items()
                 if ros_time - t.timestamp > 5.0]
        for tid in stale:
            del self.target_list[tid]
            self.get_logger().warn(f"⏰ Hedef zaman aşımı: {tid}")

    def _publish_virtual_target(self):
        """L1 sanal hedef noktasını görselleştirme için yayınla"""
        vt = self.l1_guidance.get_virtual_target()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = vt[0]
        msg.pose.position.y = vt[1]
        msg.pose.position.z = -vt[2]   # NED → ENU
        self.virtual_target_pub.publish(msg)

    def _publish_target_scores(self):
        """Hedef skor bilgilerini yayınla (debug)"""
        all_scores = self.l1_guidance.get_all_target_scores()
        if all_scores:
            scores_data = []
            for s in all_scores:
                scores_data.append({
                    'id': s.target_id,
                    'total_score': round(s.total_score, 3),
                    'distance': round(s.distance, 1),
                    'is_in_fov': s.is_in_camera_fov,
                    'is_tail': s.is_tail_position,
                })
            msg = String()
            msg.data = json.dumps(scores_data)
            self.target_scores_pub.publish(msg)

    # ========================================================================
    # DURUM LOGLAMA (2s)
    # ========================================================================

    def _durum_logla(self):
        """Periyodik durum bilgisi"""
        durum_adi = DURUM_ADLARI.get(self.durum, "?")

        if not self.px4_connected:
            self.get_logger().warn(f"[{durum_adi}] ⏳ PX4 bağlantısı yok!")
            return

        since_status = time.time() - self.last_vehicle_status_time
        if since_status > 3.0:
            self.px4_connected = False
            self.get_logger().warn(f"[{durum_adi}] ❌ PX4 bağlantısı kesildi! ({since_status:.1f}s)")
            return

        extra = ""
        if self.durum == Durum.GPS_TAKIP:
            l1_state = self.l1_guidance.get_state().name
            locked_id = self.l1_guidance.get_locked_target_id()
            extra = (f" | L1={l1_state} hedef={locked_id} "
                     f"mesafe={self.gps_target_distance:.0f}m "
                     f"hedef_sayı={len(self.target_list)} "
                     f"görsel_tespit={self.consecutive_detections}")
        elif self.durum in (Durum.GORSEL_TAKIP, Durum.KILITLENME):
            lock_dur = 0.0
            if self.visual_lock_start_time:
                lock_dur = time.time() - self.visual_lock_start_time
            extra = (f" | ex={self.vision_error_x:+.0f} ey={self.vision_error_y:+.0f} "
                     f"alan={self.vision_target_area:.0f} "
                     f"kilit_süresi={lock_dur:.1f}s")

        self.get_logger().info(
            f"[{durum_adi}] armed={self.px4_armed} "
            f"offboard={'✓' if self.px4_nav_state == NAVIGATION_STATE_OFFBOARD else '✗'} "
            f"mode={'POS' if self._control_mode_position else 'VEL'}"
            f"{extra}")

    def _check_px4_connection(self):
        """PX4 bağlantı durumu kontrolü"""
        parts = []
        parts.append(f"LocalPos:{'✓' if self._local_pos_received else '✗'}")
        parts.append(f"GlobalPos:{'✓' if self._global_pos_received else '✗'}")
        parts.append(f"Home:{'✓' if self.home_set else '✗'}")
        parts.append(f"Status:{'✓' if self.vehicle_status_received else '✗'}")
        parts.append(f"Armed:{'✓' if self.px4_armed else '✗'}")

        status_line = " | ".join(parts)

        if not self._local_pos_received or not self._global_pos_received:
            self.get_logger().warn(f"📡 PX4: {status_line}")
        elif not self.home_set:
            self.get_logger().info(f"📡 PX4: {status_line} — GPS fix bekleniyor")
        else:
            self.get_logger().info(f"📡 PX4: {status_line}")


# ============================================================================
# MAIN
# ============================================================================

def main(args=None):
    rclpy.init(args=args)

    try:
        node = JusticeNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Justice Node Hatası: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()