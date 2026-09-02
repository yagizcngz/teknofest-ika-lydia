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




class KoridorIzleyici:
    """
    §6.9: tümsekli bölüm geçilirken parkurun ortalanması gerekiyor, aracın
    tamamının aynı tümsek dizisi üzerinden geçirilmesi §9'da −5 puan.

    Tümsekler 20 cm taban / 5 cm yükseklik (Şekil 4) — LiDAR düzlemi zeminden
    55 cm'de olduğu için tümsekler taramada HİÇ görünmez, tekerlek yerleşimi
    planlanamaz. Ölçülebilen ve kuralın ilk cümlesinin istediği şey koridorda
    ortalanmak; §6.1 koridoru 3 m ve iki yanı 80 ± 10 cm bariyerli, iki duvar
    da tarama düzleminde.

    Bu sınıf yalnız ÖLÇER, direksiyona karışmaz. Gereksinim bugün sistemde
    görünmüyor: ne loglanıyor ne panoda var, dolayısıyla ortalanıp
    ortalanmadığımız ancak hakem kağıdından öğrenilirdi. Düzeltici katmanın
    gerekip gerekmediğine buradan çıkan ölçümle karar verilecek.
    """

    def __init__(self, node, lidar_yaw_rad: float = LIDAR_MONTAJ_YAW_RAD):
        self.node = node
        self._pub = node.create_publisher(Float32, ENGEBELI_SAPMA_TOPIC, 10)

        # LiDAR gövdeye dönük monte; düzeltme TF'te uygulanıyor, ham taramada
        # değil. Araç ekseni açıları bu yüzden tarama çerçevesine kaydırılır.
        self._sol_merkez = aci_sarmala(math.pi / 2.0 - lidar_yaw_rad)
        self._sag_merkez = aci_sarmala(-math.pi / 2.0 - lidar_yaw_rad)

        self._lock   = threading.Lock()
        self._aktif  = False
        self._ornek  = []      # geçerli sapmalar [m]
        self._atlanan = 0      # koridor okunamayan tarama sayısı

        node.create_subscription(LaserScan, SCAN_FILTERED_TOPIC, self._on_scan, 10)

    def basla(self):
        with self._lock:
            self._aktif   = True
            self._ornek   = []
            self._atlanan = 0

    def _on_scan(self, msg: LaserScan):
        with self._lock:
            if not self._aktif:
                return
        sol = tarama_yan_mesafe(
            list(msg.ranges), msg.angle_min, msg.angle_increment,
            self._sol_merkez, KORIDOR_PENCERE_RAD,
            msg.range_min, msg.range_max,
        )
        sag = tarama_yan_mesafe(
            list(msg.ranges), msg.angle_min, msg.angle_increment,
            self._sag_merkez, KORIDOR_PENCERE_RAD,
            msg.range_min, msg.range_max,
        )
        sapma, durum = koridor_sapmasi(
            sol, sag, KORIDOR_GENISLIGI_M, KORIDOR_TOPLAM_TOLERANS,
        )
        with self._lock:
            if durum != 'gecerli':
                self._atlanan += 1
                return
            self._ornek.append(sapma)
        self._pub.publish(Float32(data=float(sapma)))
        if abs(sapma) > KORIDOR_SAPMA_UYARI_M:
            yon = 'sağa' if sapma > 0 else 'sola'
            self.node.get_logger().warn(
                f'[KORİDOR] §6.9 merkezden {abs(sapma):.2f}m {yon} kaymış '
                f'(sol {sol:.2f}m / sağ {sag:.2f}m).',
                throttle_duration_sec=2.0,
            )

    def bitir(self):
        """Aşama sonunda özet loglar ve izlemeyi kapatır."""
        with self._lock:
            self._aktif = False
            ornek   = list(self._ornek)
            atlanan = self._atlanan

        if not ornek:
            self.node.get_logger().warn(
                f'[KORİDOR] §6.9 hiç geçerli koridor ölçümü alınamadı '
                f'({atlanan} tarama elendi) — ortalanma doğrulanamadı.'
            )
            return

        ortalama = sum(ornek) / len(ornek)
        enbuyuk  = max(ornek, key=abs)
        self.node.get_logger().info(
            f'[KORİDOR] §6.9 özet: {len(ornek)} ölçüm, ortalama sapma '
            f'{ortalama:+.2f}m, en büyük {enbuyuk:+.2f}m, '
            f'{atlanan} tarama elendi (+ sağa kayma).'
        )


# DetectionsStore → teknofest_ika.otonomi.pure_logic (rclpy bağımsız, DRY +
# test_birim.py gerçek sınıfı doğrudan import edip test eder).


# ═══════════════════════════════════════════════════════════════════════════════
# NAV2 CLIENT
# ═══════════════════════════════════════════════════════════════════════════════

