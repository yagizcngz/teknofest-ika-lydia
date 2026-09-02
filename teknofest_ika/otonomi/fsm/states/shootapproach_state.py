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

import time
import threading

from rclpy.node import Node

from std_msgs.msg import Bool, String

import smach

from teknofest_ika.otonomi.topics import (
    TARGETING_ENABLE_TOPIC, TARGETING_STATUS_TOPIC,
)
from teknofest_ika.otonomi.pure_logic import (
    DetectionsStore,
)

# §6.10: dik eğim çıkış/iniş noktalarında STOP tabelası kaçırılsa bile
# zorunlu 2s bekleme uygulanması gereken waypoint etiketleri.
DIK_EGIM_ETIKETLERI = ('DIK_EGIM_GIRIS', 'DIK_EGIM_CIKIS', 'DIK_EGIM')

# §6.9 koridor ortalamasının izlendiği aşama.
ENGEBELI_ETIKET = 'ENGEBELİ_ARAZİ'




class ShootApproachState(smach.State):
    """
    Atış pozisyonu doğrulama.

    targeting_node'u aktifleştirir (HSV+Hough+PID ile hedef kilitler).
    Servo kontrolü ve hizalama targeting_node tarafından yapılır.
    "/targeting/status" == "ALIGNED" gelince ShootState'e geçilir.
    Max 15 saniye beklenir — timeout'ta yine de ateş edilir.

    Geçişler: 'in_position' → ShootState
              'failed'      → ErrorRecoveryState
    """

    TIMEOUT_S = 15.0

    def __init__(self, node: Node, det_store: DetectionsStore):
        smach.State.__init__(
            self,
            outcomes=['in_position', 'failed', 'e_stop'],
            input_keys=['current_wp']
        )
        self.node      = node
        self.det_store = det_store

        self._targeting_lock   = threading.Lock()
        self._targeting_status = "STANDBY"
        self._targeting_enable_pub = node.create_publisher(
            Bool, TARGETING_ENABLE_TOPIC, 10
        )
        node.create_subscription(
            String, TARGETING_STATUS_TOPIC, self._on_targeting_status, 10
        )

    def _on_targeting_status(self, msg: String):
        with self._targeting_lock:
            self._targeting_status = msg.data

    def execute(self, userdata):
        if hasattr(self.node, '_fsm_state_pub'):
            self.node._fsm_state_pub.publish(String(data='SHOOT_APPROACH'))
        self.node.get_logger().info('[SHOOT_APPROACH] targeting_node aktifleştiriliyor...')

        with self._targeting_lock:
            self._targeting_status = "STANDBY"
        self._targeting_enable_pub.publish(Bool(data=True))

        deadline = time.time() + self.TIMEOUT_S
        while time.time() < deadline:
            if self.det_store.get_field('e_stop', False):
                self.node.get_logger().error('[SHOOT_APPROACH] E-STOP.')
                self._targeting_enable_pub.publish(Bool(data=False))
                return 'e_stop'

            with self._targeting_lock:
                status = self._targeting_status

            if status == 'ALIGNED':
                self.node.get_logger().info('[SHOOT_APPROACH] Nişan tamam ✓')
                self._targeting_enable_pub.publish(Bool(data=False))
                return 'in_position'

            self.node.get_logger().debug(
                f'[SHOOT_APPROACH] targeting={status}',
                throttle_duration_sec=2.0,
            )
            time.sleep(0.05)

        self.node.get_logger().warn('[SHOOT_APPROACH] Timeout — yine de ateş ediliyor.')
        self._targeting_enable_pub.publish(Bool(data=False))
        return 'in_position'


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 4 — SHOOT
# ═══════════════════════════════════════════════════════════════════════════════

