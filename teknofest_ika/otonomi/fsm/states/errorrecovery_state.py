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




class ErrorRecoveryState(smach.State):
    """
    Hata kurtarma. Bir aşama başarısız olduğunda buraya düşülür.

    Aşama MAX_RETRIES kez denenir; hâlâ olmuyorsa o aşamadan VAZGEÇİLİR ve
    sıradaki waypoint'e geçilir. Görev iptal edilmez: tek bir istasyonda
    takılmak koşunun tamamını götürüyordu — 5 puanlık bir aşama uğruna
    koniler (50), kayar engel (50), atış (50), hızlanma (25) ve sıralamaya
    giren koşu süresi (100) kaybediliyordu.

    Vazgeçmek §9 pası DEĞİLDİR: pas, takımın parkura girip aracı aşamanın
    sonuna elle taşımasıdır ve o aşamanın puanını eksiye yazar. Burada
    yapılan yalnız "bu hedefe ulaşmayı bırak, sıradakine nişan al" —
    istasyonlar tek koridorda arka arkaya dizili olduğu için araç engelin
    içinden geçmeyi fiilen denemeye devam eder.

    Görevi bitiren iki yol buraya HİÇ uğramaz: E-STOP ('e_stop' çıktısı) ve
    §6.12 süre dolması ('sure_doldu'). İkisi de doğrudan MISSION_ABORT'a
    gider — E-STOP'un kurtarma denemesi harcaması aracı saniyelerce
    duraklatıyordu.

    Sayaç userdata'da tutulur; NavigateState bir aşamayı tamamladığında
    sıfırlar (bütçe aşama başınadır, görev başına değil).

    Geçişler: 'recovered'     → NavigateState (aynı waypoint tekrar)
              'sonraki_asama' → NavigateState (bu aşamadan vazgeçildi)
    """

    # Her deneme koşu saatinden bir aşama bütçesi yiyor; koşu süresi 100
    # puanla tablodaki en büyük kalem olduğu için deneme sayısı düşük tutuldu.
    MAX_RETRIES = 2

    def __init__(self, node: Node):
        smach.State.__init__(
            self,
            outcomes=['recovered', 'sonraki_asama'],
            input_keys=['wp_index', 'retry_count', 'waypoints'],
            output_keys=['wp_index', 'retry_count']
        )
        self.node = node
        self.birakilan = 0          # bu koşuda vazgeçilen aşama sayısı

    def execute(self, userdata):
        if hasattr(self.node, '_fsm_state_pub'):
            self.node._fsm_state_pub.publish(String(data='ERROR_RECOVERY'))
        sayac = userdata.retry_count + 1
        userdata.retry_count = sayac
        self.node.get_logger().warn(
            f'[ERROR_RECOVERY] Deneme {sayac}/{self.MAX_RETRIES}'
        )

        if _kurtarma_karari_pure(sayac, self.MAX_RETRIES) == 'sonraki':
            idx   = userdata.wp_index
            wpler = getattr(userdata, 'waypoints', None) or []
            etiket = (wpler[idx].get('label', f'#{idx}')
                      if idx < len(wpler) else f'#{idx}')
            self.birakilan += 1
            self.node.get_logger().error(
                f'[ERROR_RECOVERY] {etiket}: {self.MAX_RETRIES} deneme de '
                f'başarısız — bu aşamadan vazgeçilip sıradakine geçiliyor '
                f'(bu koşuda bırakılan aşama: {self.birakilan}). '
                'Görev SÜRÜYOR; bu bir §9 pası değildir.'
            )
            userdata.wp_index    = idx + 1
            userdata.retry_count = 0
            return 'sonraki_asama'

        # Başarısız olan waypoint TEKRAR denenir. wp_index navigasyon
        # başarısızlığında henüz artmamıştır; indeksi geri almak bir ÖNCEKİ
        # (tamamlanmış) aşamaya döner ve varışta o aşamanın tipi yeniden
        # okunur — ATIS_BOLGESI'ne dönülürse lazer ikinci kez ateşlenir,
        # HIZLANMA_PARKURU'na dönülürse 30 m'lik koşu baştan başlar.
        time.sleep(2.0)
        return 'recovered'


# ═══════════════════════════════════════════════════════════════════════════════
# WAYPOINT YÜKLEYİCİ
# ═══════════════════════════════════════════════════════════════════════════════

