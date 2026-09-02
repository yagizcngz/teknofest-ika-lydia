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




class HizlanmaState(smach.State):
    """
    §6.11 Hızlanma Parkuru — Nav2 bypass, doğrudan /cmd_vel → mod_yoneticisi.

    NavigateState buraya waypoint'in `type: hizlanma` alanıyla geçer, tabela
    tespitiyle değil: hızlanma parkuru koordinattan biliniyor ve tek bir YOLO
    karesine bağlanamayacak kadar pahalı (§6.11 ilk altı takıma puan veriyor).
    Tabela_11 bu state'te yalnız `terrain_adapter`'ın `fast` profilini
    tetikler; Tabela_11_son ise aşağıdaki erken çıkışı.
    Nav2 path planner devre dışı — düz pistte gereksiz overhead.
    RC override hâlâ çalışır (mod_yoneticisi /cmd_vel'i dinlemeye devam eder).

    /cmd_vel'in ikinci yayıncısı Nav2'nin velocity_smoother'ıdır. Girdisi
    kesildiğinde smoother susmaz: araç hareket hâlindeyse komutu sıfıra çekip
    max_decel ile yavaşlayan bir rampa yayınlar, ancak sıfıra ulaşınca
    (stopped_) yayını tamamen keser. Bu state'e araç HAREKET HÂLİNDEYKEN
    girilirse iki yayıncı yaklaşık bir saniye boyunca aynı topic'e yazar ve
    mod_yoneticisi son geleni geçirdiği için gaz titrer. Normal akışta sorun
    çıkmaz: NavigateState hedefe varınca Nav2 aracı zaten durdurur ve smoother
    susmuş olur. Buraya "araç yürürken" atlanacak bir yol eklenirse önce
    smoother'ın susması beklenmelidir.

    Hız profili:
      0  .. OLCUM_MESAFE (30m) → MAX_HIZ — puanlanan bölüm sonuna kadar tam gaz
      çizgide                  → 0 komutu, fren zinciri devralır
      30 .. +DURMA_MESAFE(10m) → duruş izlenir, gerçekleşen mesafe loglanır
      Erken çıkış: Tabela_11_son tespiti VEYA timeout

    Geçişler: 'completed' → NavigateState
              'failed'    → ErrorRecoveryState
    """

    # Ölçüm (2026-07-28): gaz voltajı = 0.90 + hız[m/s]; tam gaz 2.84 V ≈
    # 1.94 m/s. Eski 3.0 değeri seri_kopru.MAX_HIZ_MS donanım kelepçesiydi,
    # aracın ulaşabildiği bir hız değil — 3.90 V ister, DAC/sürücü doyar.
    MAX_HIZ          = 1.90   # m/s — ölçülen tam gazın hemen altı
    # Şartname §6.11 iki ayrı mesafe tanımlıyor: 30 m puanlanan hızlanma
    # bölümü, ardından 10 m emniyetli durma payı. Fren payın içinde yapılır;
    # ölçülen bölümde yavaşlamak yalnız ilk altı takıma puan veren bir
    # kalemde sıralama kaybettirir. 1.90 m/s'den 10 m'de durmak 0.18 m/s²
    # ister, yani pay fazlasıyla geniş.
    OLCUM_MESAFE     = 30.0   # m — puanlanan bölüm
    DURMA_MESAFE     = 10.0   # m — çizgi sonrası emniyetli durma payı
    DURMA_HIZ_ESIGI  = 0.05   # m/s — bunun altı "durdu" sayılır
    # Payı tam hızda katetmek 5.3 s sürer, duruş bundan kısadır. Bu guard
    # odometri hareket halindeyken donarsa döngünün asılı kalmasını önler.
    DURMA_TIMEOUT_S  = 10.0   # s
    # Timeout mesafeden türetilir. Sabit 15 s, 30 m için 2.0 m/s ORTALAMA
    # gerektiriyordu; ölçülen tepe hız 1.94 m/s olduğundan parkur ivmelenme
    # süresi sıfır olsa bile her koşuda timeout'la bitiyordu. Bu bir kontrol
    # parametresi değil, donanım arızasına karşı üst limit: en kötü hâlde
    # ortalama hızın MAX_HIZ'in %40'ı olduğu varsayılır (kalkış sürtünmesi +
    # durma payı).
    TIMEOUT_S        = (OLCUM_MESAFE + DURMA_MESAFE) / (MAX_HIZ * 0.40)
    CMD_HZ           = 10.0   # Hz
    # Odom donmuşsa (enkoder yok/kopuk) /odom sabit (0,0) yayınlar; dist hep 0
    # kalır, profil hep MAX_HIZ verir ve araç timeout'a kadar tam gaz gider.
    ODOM_ILERLEME_S  = 4.0    # s — bu süre sonunda ilerleme yoksa iptal
    ODOM_ILERLEME_M  = 0.20   # m
    ODOM_BAYATLAMA_S = 1.0    # s — /odom mesajı bu süre gelmezse iptal

    def __init__(self, node: Node, det_store: DetectionsStore,
                 saat: 'MisyonSaati'):
        smach.State.__init__(self, outcomes=['completed', 'failed', 'e_stop'])
        self.saat = saat
        self.node      = node
        self.det_store = det_store

        self._cmd_pub = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self._lock     = threading.Lock()
        self._pos      = None   # (x, y) — son odom konumu
        self._hiz      = 0.0    # m/s — durma payının izlenmesi için
        self._son_odom = 0.0    # wall-clock — son /odom mesajının geliş anı

        node.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._pos = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            )
            self._hiz = msg.twist.twist.linear.x
            self._son_odom = time.time()

    def _hiz_oku(self) -> float:
        with self._lock:
            return self._hiz

    def _odom_yasi(self) -> float:
        with self._lock:
            return time.time() - self._son_odom

    def _mesafe(self, baslangic) -> float:
        with self._lock:
            if self._pos is None:
                return 0.0
            dx = self._pos[0] - baslangic[0]
            dy = self._pos[1] - baslangic[1]
        return math.sqrt(dx * dx + dy * dy)

    def execute(self, userdata):
        if hasattr(self.node, '_fsm_state_pub'):
            self.node._fsm_state_pub.publish(String(data='HIZLANMA'))

        # Odom başlangıç konumunu al (maks 2s bekle)
        deadline_init = time.time() + 2.0
        while time.time() < deadline_init:
            with self._lock:
                if self._pos is not None:
                    baslangic = self._pos
                    break
            time.sleep(0.05)
        else:
            self.node.get_logger().error('[HIZLANMA] Odom verisi yok — iptal.')
            return 'failed'

        self.node.get_logger().info(
            f'[HIZLANMA] Başladı. Hedef: {self.OLCUM_MESAFE:.0f}m @ {self.MAX_HIZ:.1f} m/s'
        )

        twist    = Twist()
        dt       = 1.0 / self.CMD_HZ
        baslama  = time.time()
        deadline = baslama + self.TIMEOUT_S
        sonuc    = 'completed'

        while rclpy.ok():
            if self.det_store.get_field('e_stop', False):
                self.node.get_logger().error('[HIZLANMA] E-STOP — durduruluyor.')
                sonuc = 'e_stop'
                break

            if self.det_store.get_field('manual_mod', False):
                self.node.get_logger().warn(
                    '[HIZLANMA] Manuel mod — RC devraldı, bekleniyor.',
                    throttle_duration_sec=2.0,
                )
                twist.linear.x = 0.0
                self._cmd_pub.publish(twist)
                time.sleep(0.1)
                # Manuel duraklama timeout bütçesinden sayılmaz; sayılsaydı
                # deadline kontrolü bu daldan sonra geldiği için timeout hiç
                # dolmaz ve RC manuelde kaldığı sürece döngü sonsuza giderdi.
                deadline += 0.1
                baslama  += 0.1
                continue

            if time.time() > deadline:
                self.node.get_logger().warn('[HIZLANMA] Timeout — durduruluyor.')
                break

            # §6.12: koşu saati manuel duraklamalarla uzayan aşama bütçesinden
            # bağımsız işler; dolduysa 30 m tamamlanmamış olsa da fren.
            if self.saat.doldu():
                self.node.get_logger().error(
                    '[HIZLANMA] §6.12 koşu süresi doldu — fren.'
                )
                break

            # Odom sağlığı: kopuk/donmuş enkoder mesafeyi hep 0 gösterir,
            # profil hep MAX_HIZ verir ve araç timeout'a kadar tam gaz gider.
            yas = self._odom_yasi()
            if yas > self.ODOM_BAYATLAMA_S:
                self.node.get_logger().error(
                    f'[HIZLANMA] /odom {yas:.1f}s bayat — iptal.'
                )
                sonuc = 'failed'
                break

            dist = self._mesafe(baslangic)

            if (time.time() - baslama > self.ODOM_ILERLEME_S
                    and dist < self.ODOM_ILERLEME_M):
                self.node.get_logger().error(
                    f'[HIZLANMA] {self.ODOM_ILERLEME_S:.0f}s tam gazda ilerleme '
                    f'{dist:.2f}m — odometri donmuş veya araç hareket etmiyor, iptal.'
                )
                sonuc = 'failed'
                break

            if self.det_store.get_field('hizlanma_bitti', False):
                self.node.get_logger().info(
                    f'[HIZLANMA] Tabela_11_son tespit ({dist:.1f}m) — fren.'
                )
                break

            if dist >= self.OLCUM_MESAFE:
                self.node.get_logger().info(
                    f'[HIZLANMA] {self.OLCUM_MESAFE:.0f}m tamamlandı — fren.'
                )
                break

            # Hız profili — pure_logic.hizlanma_hiz_profili (test_birim.py
            # bu fonksiyonu doğrudan test eder).
            hiz = hizlanma_hiz_profili(dist, self.MAX_HIZ, self.OLCUM_MESAFE)

            twist.linear.x  = hiz
            twist.angular.z = 0.0
            self._cmd_pub.publish(twist)

            self.node.get_logger().info(
                f'[HIZLANMA] dist={dist:.1f}m  hız={hiz:.1f}m/s',
                throttle_duration_sec=0.5,
            )
            time.sleep(dt)

        # Durma payı — §6.11 bitiş çizgisinin ötesinde 10 m veriyor ve o pay
        # içinde duramayan araca ceza yazıyor. Sıfır komutunu birkaç kez
        # yayınlayıp çıkmak duruşu kimsenin izlemediği anlamına gelirdi:
        # kontrol bir sonraki state'e geçer, gerçekleşen durma mesafesi hiç
        # bilinmez ve ceza ancak hakem masasında öğrenilir.
        twist.linear.x  = 0.0
        twist.angular.z = 0.0
        cizgi      = self._mesafe(baslangic)
        durma_basi = time.time()
        durum      = 'devam'

        while rclpy.ok():
            self._cmd_pub.publish(twist)
            asilan = max(0.0, self._mesafe(baslangic) - cizgi)
            durum  = durma_degerlendir(
                asilan, self._hiz_oku(), self.DURMA_MESAFE,
                self.DURMA_HIZ_ESIGI,
            )
            if durum != 'devam':
                break
            if time.time() - durma_basi > self.DURMA_TIMEOUT_S:
                durum = 'butce_asildi'
                break
            time.sleep(dt)

        if durum == 'butce_asildi':
            self.node.get_logger().error(
                f'[HIZLANMA] §6.11 durma payı aşıldı — {asilan:.1f}m/'
                f'{self.DURMA_MESAFE:.0f}m, hız {self._hiz_oku():.2f} m/s. '
                f'Ceza puanı riski.'
            )
        else:
            self.node.get_logger().info(
                f'[HIZLANMA] {asilan:.1f}m içinde durdu '
                f'(pay {self.DURMA_MESAFE:.0f}m).'
            )

        self.node.get_logger().info(f'[HIZLANMA] Tamamlandı → {sonuc}.')
        return sonuc


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 6 — RAMPA  (§6.10 dik eğim, Nav2 baypas)
# ═══════════════════════════════════════════════════════════════════════════════

