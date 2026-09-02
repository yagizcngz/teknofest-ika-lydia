#!/usr/bin/env python3
"""
misyon_fsm.py — LYDİA İKA Görev Yöneticisi (SMACH FSM)
=========================================================
v2.0 — Ekip Protokol v1.0 ile uyumlu

NE YAPAR:
  Parkur boyunca görev sırasını yönetir. Nav2'ye "şimdi şu noktaya git"
  komutunu verir. Atış bölgesinde durur, nişan alır, ateş eder.

  Görüntü ekibinden /ika/detections (JSON) alır:
    - kayar_yon → KAYAR_ENGEL geçiş kararı
    - tabela    → FSM bilgi (terrain_adapter halleder)

  NOT — hedef_var / hedef_hata_x / hedef_hata_y bu JSON'da hesaplanıp
  taşınıyor ama FSM bunları HİÇ okumuyor; bilinçli bir tasarım kararı
  (ekip: "hedef tahtasını tanıyacak, nişanlama yapmayacak"). YOLO'nun
  hedef_tahtasi tespiti sadece kayıt/bilgi amaçlıdır. Gerçek nişan alma
  tamamen ayrı ve YOLO'dan bağımsızdır — bkz. aşağıdaki ATIS waypoint notu.

PROTOKOL ENTEGRASYONU (Ekip Protokol v1.0):
  Giriş : /ika/detections (JSON String) — Görüntü İşleme → Otomasyon
  Çıkış : Nav2 NavigateToPose action   — Otomasyon navigasyon
  Çıkış : /shoot_command (Bool)        — Otomasyon → Gömülü Sistemler köprüsü
           seri_kopru.py bu komutu LAZER,1 / LAZER,0 seri komutuna çevirir

ÖZEL DURUM MANTIĞI:
  DIK_EGIM waypoint : Şartname gereği hedefe varışta 2 saniye dur, sonra devam.
  KAYAR_ENGEL waypoint: kayar_yon != 'bilinmiyor' olana kadar bekle (max 10s).
  ATIS waypoint    : targeting_node etkinleştirilir (kendi HSV+Hough+PID'i
                      ile hedef kilitler, YOLO hedef verisi kullanılmaz),
                      "/targeting/status" == "ALIGNED" gelince ateş edilir.

DURUM DİYAGRAMI:
  [IDLE] → [NAVIGATE] ─┬─ type=shoot ──→ [SHOOT_APPROACH] → [SHOOT] ─┐
                       │                                             │
                       ├─ type=hizlanma → [HIZLANMA] ────────────────┤
                       │                                             │
                       ├─ type=rampa ──→ [RAMPA] ────────────────────┤
                       │   (§6.10 Nav2 baypas: doğrudan /cmd_vel)     │
                       ├─ liste bitti ──→ [MISSION_COMPLETE] → GOREV_TAMAMLANDI
                       │                                             │
                       └───────────── [NAVIGATE] ←───────────────────┘
                       ↓ failed                 ↑ recovered (aynı wp)
                 [ERROR_RECOVERY] ──────────────┤
                       ↓ 2 deneme               ↑ sonraki_asama (wp+1)
                       └────────────────────────┘

  E-STOP → [MISSION_ABORT]         §6.12 süre doldu → [MISSION_ABORT]

  Görevi bitiren yollar YALNIZ ikisidir: E-STOP ve §6.12 süre dolması.
  İkisi de kurtarmaya uğramadan doğrudan MISSION_ABORT'a gider.

  Bir aşama 2 denemede de olmazsa o aşamadan VAZGEÇİLİR ve sıradaki
  waypoint'e geçilir — görev sürer. Eskiden görev iptal ediliyordu ve
  5 puanlık bir istasyonda takılmak, aşağı akıştaki koniler (50), kayar
  engel (50), atış (50), hızlanma (25) ve sıralamaya giren koşu süresi
  (100) puanlarının tamamını götürüyordu. Vazgeçmek §9 pası DEĞİLDİR:
  pas, aracın aşama sonuna elle taşınmasıdır ve puanı eksiye yazar.

  MISSION_COMPLETE ve MISSION_ABORT AYRI durumlardır: ilki
  /mission_status = 'COMPLETE', ikincisi 'ABORTED' yayınlar. Kurtarma
  bütçesi (2) aşama başınadır — NavigateState her tamamlanan waypoint'te
  sayacı sıfırlar.

BAĞIMLILIKLAR:
  sudo apt install ros-humble-smach ros-humble-smach-ros
  pip3 install smach --break-system-packages

ÇALIŞTIRMA:
  ros2 run teknofest_ika misyon_fsm

TEST (kamerasız):
  ros2 topic pub /mission_start std_msgs/msg/Bool "data: true" --once
  ros2 topic pub /ika/detections std_msgs/msg/String \
    'data: "{\"tabela\":9,\"hedef_var\":true,\"hedef_hata_x\":2.0,\"hedef_hata_y\":1.0,\"kayar_yon\":\"sol\",\"fps\":30}"' \
    --rate 10
"""

import json
import math
import time
import threading
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, Float32, String, UInt8
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from nav2_msgs.action import NavigateToPose

import smach

from teknofest_ika.otonomi.topics import (
    FSM_STATE_TOPIC, DETECTIONS_TOPIC,
    MOVING_OBS_DIR_TOPIC, E_STOP_TOPIC, MOD_AKTIF_TOPIC,
    MISSION_START_TOPIC, MISYON_AKTIF_TOPIC,
    TARGETING_ENABLE_TOPIC, TARGETING_STATUS_TOPIC,
    SHOOT_CMD_TOPIC, SHOOT_RESULT_TOPIC, LASER_FIRE_DURATION,
    MISSION_STATUS_TOPIC, MISYON_WP_INDEX_TOPIC,
    CMD_VEL_TOPIC, ODOM_TOPIC, EKF_ODOM_TOPIC, RAMP_STOP_DURATION,
    MISYON_KALAN_SURE_TOPIC, KOSU_SURESI_S, PAS_HAKKI, PAS_GECILEMEZ,
    SCAN_FILTERED_TOPIC, ENGEBELI_SAPMA_TOPIC, KORIDOR_GENISLIGI_M,
    IMU_TOPIC, IMU_PITCH_RAMP_THRESHOLD,
    KORIDOR_SAPMA_UYARI_M, KORIDOR_TOPLAM_TOLERANS, KORIDOR_PENCERE_RAD,
    KAYAN_HEDEF_PERIYOT_S, KAYAN_HEDEF_FRAME, KAYAN_HEDEF_YOK_SINIR,
    KAYAN_ODOM_BAYATLAMA_S, LIDAR_MONTAJ_YAW_RAD,
)
from teknofest_ika.otonomi.pure_logic import (
    DetectionsStore, stop_check as _stop_check_pure, hizlanma_hiz_profili,
    durma_degerlendir, tarama_yan_mesafe, koridor_sapmasi, aci_sarmala,
    kosu_butcesi, pas_verilebilir as _pas_verilebilir_pure,
    kurtarma_karari as _kurtarma_karari_pure,
    quat_to_roll_pitch_deg, rollback_riskli, rampa_faz_gecisi,
    koridor_merkez_cizgisi, kayan_hedef, ic_duvar_hedefi,
    hedefleme_modu_sec, kayan_hedef_karari,
    quat_yaw, arac_hedefini_odoma_tasi,
)

# §6.10: dik eğim çıkış/iniş noktalarında STOP tabelası kaçırılsa bile
# zorunlu 2s bekleme uygulanması gereken waypoint etiketleri.
DIK_EGIM_ETIKETLERI = ('DIK_EGIM_GIRIS', 'DIK_EGIM_CIKIS', 'DIK_EGIM')

# §6.9 koridor ortalamasının izlendiği aşama.
ENGEBELI_ETIKET = 'ENGEBELİ_ARAZİ'




class Nav2Client:
    """
    Nav2 NavigateToPose action'ını saran yardımcı sınıf.
    SMACH state'leri bu sınıf üzerinden Nav2 ile konuşur.
    """

    def __init__(self, node: Node):
        self.node = node
        self._client = ActionClient(
            node,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=ReentrantCallbackGroup()
        )
        # Kayan hedef sürüşünde etkin goal'ın handle'ı — iptal için saklanır.
        self._son_handle = None

    def go_to(self, x: float, y: float, yaw: float,
              timeout_sec: float = 120.0,
              stop_check_fn=None) -> str:
        """
        Verilen (x, y, yaw) noktasına git. Bloklar.
        Dönüş: 'success' | 'failed' | 'timeout' | 'stop_requested'

        stop_check_fn: callable → bool. True döndürünce goal iptal edilir,
        'stop_requested' döner. NavigateState §6.10 STOP mantığı için kullanır.
        """
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(
                'Nav2 /navigate_to_pose servisi bulunamadı!'
            )
            return 'failed'

        goal = NavigateToPose.Goal()
        goal.pose = self._build_pose(x, y, yaw)

        self.node.get_logger().info(
            f'Nav2 hedef → ({x:.2f}, {y:.2f}, yaw={yaw:.2f} rad) '
            f'timeout={timeout_sec:.0f}s'
        )

        deadline = time.time() + timeout_sec

        future = self._client.send_goal_async(goal)
        while not future.done():
            if time.time() > deadline:
                self.node.get_logger().warn('Nav2 hedef gönderme timeout.')
                return 'timeout'
            if stop_check_fn and stop_check_fn():
                return 'stop_requested'
            time.sleep(0.05)

        try:
            goal_handle = future.result()
        except Exception as e:
            self.node.get_logger().error(f'Nav2 hedef gönderilemedi: {e}')
            return 'failed'

        if not goal_handle.accepted:
            self.node.get_logger().warn('Nav2 hedefi reddetti.')
            return 'failed'

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if time.time() > deadline:
                self.node.get_logger().warn(
                    f'Nav2 aşama timeout ({timeout_sec:.0f}s) — hedef iptal ediliyor.'
                )
                goal_handle.cancel_goal_async()
                return 'timeout'
            if stop_check_fn and stop_check_fn():
                goal_handle.cancel_goal_async()
                return 'stop_requested'
            time.sleep(0.05)

        try:
            result = result_future.result()
        except Exception as e:
            self.node.get_logger().error(f'Nav2 sonuç alınamadı: {e}')
            return 'failed'

        success = (result.status == 4)  # GoalStatus.STATUS_SUCCEEDED = 4

        if success:
            self.node.get_logger().info(f'Hedefe ulaşıldı ✓ ({x:.2f}, {y:.2f})')
        else:
            self.node.get_logger().warn(
                f'Hedefe ulaşılamadı. Status={result.status}'
            )
        return 'success' if success else 'failed'

    def hedef_gonder(self, x: float, y: float, yaw: float,
                     frame_id: str = KAYAN_HEDEF_FRAME) -> bool:
        """
        Bloklamadan hedef gönderir; gönderilen hedef öncekini preempt eder.
        Kayan hedef sürüşü için: hedef sürekli yenilendiği için `go_to`'nun
        "gönder ve sonucu bekle" akışı burada kullanılamaz.

        `frame_id` varsayılan olarak araç çerçevesidir. Hedef taramadan
        üretildiği için araç çerçevesinde doğar; Nav2 kabul anında TF ile
        global çerçeveye çevirir. Böylece `map` çerçevesinin kayması hedefe
        birikmez — her döngüde hedef zaten yeniden doğuyor.
        """
        if not self._client.server_is_ready():
            if not self._client.wait_for_server(timeout_sec=2.0):
                self.node.get_logger().error(
                    'Nav2 /navigate_to_pose hazır değil — kayan hedef gönderilemiyor.'
                )
                return False

        goal = NavigateToPose.Goal()
        goal.pose = self._build_pose(x, y, yaw, frame_id)

        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._hedef_kabul_edildi)
        return True

    def _hedef_kabul_edildi(self, future):
        """send_goal_async sonucu — iptal edebilmek için handle saklanır."""
        try:
            handle = future.result()
        except Exception as e:
            self.node.get_logger().warn(f'Nav2 kayan hedef gönderilemedi: {e}')
            return
        if handle.accepted:
            self._son_handle = handle
        else:
            self.node.get_logger().warn('Nav2 kayan hedefi reddetti.')

    def iptal(self):
        """Etkin kayan hedefi iptal eder. Nav2 iptalde aracı durdurur."""
        handle = self._son_handle
        self._son_handle = None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as e:
                self.node.get_logger().warn(f'Nav2 hedef iptali başarısız: {e}')

    def _build_pose(self, x: float, y: float, yaw: float,
                    frame_id: str = 'map') -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp    = self.node.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        # yaw → quaternion (2D: sadece z ve w)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose


# ═══════════════════════════════════════════════════════════════════════════════
# KOŞU SAATİ
# ═══════════════════════════════════════════════════════════════════════════════

