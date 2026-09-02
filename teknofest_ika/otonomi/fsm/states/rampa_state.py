from teknofest_ika.otonomi.fsm.helpers.misyon_saati import MisyonSaati
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

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import smach

from teknofest_ika.otonomi.topics import (
    CMD_VEL_TOPIC, ODOM_TOPIC,
    RAMP_STOP_DURATION, IMU_TOPIC, IMU_PITCH_RAMP_THRESHOLD,
)
from teknofest_ika.otonomi.pure_logic import (
    DetectionsStore, quat_to_roll_pitch_deg, rollback_riskli,
    rampa_faz_gecisi,
)

# §6.10: dik eğim çıkış/iniş noktalarında STOP tabelası kaçırılsa bile
# zorunlu 2s bekleme uygulanması gereken waypoint etiketleri.
DIK_EGIM_ETIKETLERI = ('DIK_EGIM_GIRIS', 'DIK_EGIM_CIKIS', 'DIK_EGIM')

# §6.9 koridor ortalamasının izlendiği aşama.
ENGEBELI_ETIKET = 'ENGEBELİ_ARAZİ'




class RampaState(smach.State):
    """
    §6.10 dik eğimi Nav2'ye sormadan sürer. HizlanmaState ile aynı desen:
    doğrudan /cmd_vel, planlayıcı devre dışı, güvenlik zinciri (mod_yoneticisi
    E-STOP sıfırlaması + speed_limit kırpması + ackermann_converter kelepçeleri)
    aynen mirasta kalır.

    NEDEN BAYPAS
    ─────────────────────────────────────────────────────────────────────────
    Rampada planlayıcının verecek kararı yok: koridor 3 m, direksiyon kararı
    yok, ve rampanın ayak izinde bariyer yok (parkur CAD'inde dört bariyerin
    dördü de giriş/çıkışta). Buna karşılık Nav2 rampada iki ayrı yanlış
    üretiyor:

      1. 0,55 m'deki yatay tarama düzlemi %45 eğimli yüzeyi tabanından
         0,55/0,45 = 1,22 m içeride keser ve orada DUVAR okur. 2B taramada
         eğimli zeminle dik duvar geometrik olarak ayırt edilemez.
      2. base_footprint→base_link statik (0 0 0) ve ekf two_d_mode roll/pitch'i
         her döngüde sıfırlıyor: TF ağacı aracın eğildiğini hiç bilmez, rampa
         üzerindeyken her tarama dönüşü düz zemine izdüşürülür.

    FAZ SİNYALİ PİTCH, KONUM DEĞİL
    ─────────────────────────────────────────────────────────────────────────
    Waypoint koordinatları kayabilir ve odometri sapar; ama araç eğimdeyse IMU
    bunu mutlak olarak söyler. Karar pure_logic.rampa_faz_gecisi'nde, pitch'in
    MUTLAK değeriyle — böylece aynı mantık hem tırmanışta hem inişte çalışır.
    Aşamanın tırmanış mı iniş mi olduğu waypoint etiketinden okunur, çünkü
    rampanın tepesinde (atış noktası, rampa ayak izinin tam ortası) pitch iki
    durumda da ~0'dır ve tek başına ayırt etmez.

    DURUŞ
    ─────────────────────────────────────────────────────────────────────────
    §6.10 iki saniyelik duruş istiyor. Sayaç, sıfır komutu basıldığı an değil
    araç GERÇEKTEN durduktan sonra başlar: hakemin kronometresi aracı ölçüyor,
    bizim niyetimizi değil. Yavaşlama velocity_smoother'ın ivme rampasına tabi
    olduğu için ikisi arasında ölçülmemiş bir gecikme var.

    Geçişler: 'completed' → NavigateState
              'failed'    → ErrorRecoveryState
              'e_stop'    → MissionAbortState
    """

    CMD_HZ            = 10.0    # Hz
    PITCH_ESIK_DEG    = IMU_PITCH_RAMP_THRESHOLD
    DURMA_HIZ_ESIGI   = 0.05    # m/s — bunun altı "durdu" sayılır
    DURMA_TIMEOUT_S   = 8.0     # s — bu sürede duramazsa yine de devam et
    YAKLASMA_MAX_M    = 3.0     # m — bu kadar sürüldüğü hâlde eğim yoksa iptal
    EGIM_MAX_M        = 6.0     # m — CAD: tırmanış koşusu 4,13 m
    # Donanım arızası korumaları — HizlanmaState'ten. Rampada daha kritik:
    # kopuk enkoderle mesafe hep 0 okunur, faz hiç bitmez ve araç eğimde
    # timeout'a kadar gaz vermeye devam eder.
    ODOM_BAYATLAMA_S  = 1.0     # s
    ODOM_ILERLEME_S   = 4.0     # s
    ODOM_ILERLEME_M   = 0.20    # m
    # Geri kayma bu süre boyunca sürerse tahrik yetmiyor demektir; daha fazla
    # gaz vermek çözmez. anti_rollback düğümü ackermann_converter üzerinden
    # zaten karşı komut basıyor — burada onunla yarışılmaz, yalnız iptal edilir.
    GERI_KAYMA_S      = 1.5     # s

    def __init__(self, node: Node, det_store: DetectionsStore,
                 saat: 'MisyonSaati',
                 tirmanis_hiz: float = 0.6, inis_hiz: float = 0.4):
        smach.State.__init__(self, outcomes=['completed', 'failed', 'e_stop'],
                             input_keys=['current_wp'])
        self.node          = node
        self.det_store     = det_store
        self.saat          = saat
        self.tirmanis_hiz  = tirmanis_hiz
        self.inis_hiz      = inis_hiz



        self._cmd_pub = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self._lock     = threading.Lock()
        self._pos      = None
        self._hiz      = 0.0
        self._pitch    = 0.0
        self._son_odom = 0.0

        node.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)
        # IMU BEST_EFFORT yayınlanıyor (anti_rollback ile aynı profil).
        # RELIABLE abone olunursa eşleşme kurulmaz ve callback HİÇ çağrılmaz —
        # hata basılmaz, pitch sonsuza kadar 0 kalır ve faz hiç değişmez.
        node.create_subscription(
            Imu, IMU_TOPIC, self._on_imu,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    # ── Sensör callback'leri ────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self._hiz = msg.twist.twist.linear.x
            self._son_odom = time.time()

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        _, pitch = quat_to_roll_pitch_deg(q.w, q.x, q.y, q.z)
        with self._lock:
            self._pitch = pitch

    def _oku(self):
        with self._lock:
            return self._pos, self._hiz, self._pitch, self._son_odom

    def _timeout(self, hedef_hiz: float) -> float:
        """Aşama üst sınırı — HizlanmaState ile aynı gerekçe.

        Sabit bir sayı hıza göre ya hiç dolmaz ya her koşuda dolar. Bu bir
        kontrol parametresi değil, donanım arızasına karşı üst sınır: en kötü
        hâlde ortalama hızın hedefin %40'ı olduğu varsayılır (kalkış
        sürtünmesi, eğimde tork düşüşü), üstüne duruş payı eklenir. Tırmanış
        ve iniş farklı hızlarda sürüldüğü için her faz kendi bütçesini alır.
        """
        return ((self.YAKLASMA_MAX_M + self.EGIM_MAX_M)
                / max(0.05, hedef_hiz * 0.40)
                + self.DURMA_TIMEOUT_S + RAMP_STOP_DURATION)

    @staticmethod
    def _mesafe(a, b):
        if a is None or b is None:
            return 0.0
        return math.hypot(b[0] - a[0], b[1] - a[1])

    # ── Duruş ───────────────────────────────────────────────────────────────

    def _dur_ve_bekle(self, sure_s: float, etiket: str) -> str:
        """Sıfır komutu basar, GERÇEKTEN durmayı bekler, sonra sure_s sayar."""
        twist = Twist()
        durma_basi = time.time()
        durdu_mu   = False

        while rclpy.ok():
            self._cmd_pub.publish(twist)
            if self.det_store.get_field('e_stop', False):
                return 'e_stop'
            _, hiz, _, _ = self._oku()
            if abs(hiz) < self.DURMA_HIZ_ESIGI:
                durdu_mu = True
                break
            if time.time() - durma_basi > self.DURMA_TIMEOUT_S:
                self.node.get_logger().error(
                    f'[RAMPA] {etiket}: {self.DURMA_TIMEOUT_S:.0f}s içinde '
                    f'durulamadı (hız={hiz:.2f} m/s) — §6.10 duruşu şüpheli.'
                )
                break
            time.sleep(1.0 / self.CMD_HZ)

        if durdu_mu:
            self.node.get_logger().info(
                f'[RAMPA] {etiket}: durdu ({time.time() - durma_basi:.1f}s), '
                f'§6.10 {sure_s:.0f}s bekleme başlıyor.'
            )

        bekleme_bitis = time.time() + sure_s
        while time.time() < bekleme_bitis:
            self._cmd_pub.publish(twist)
            if self.det_store.get_field('e_stop', False):
                return 'e_stop'
            time.sleep(1.0 / self.CMD_HZ)
        return 'ok'

    # ── Ana akış ────────────────────────────────────────────────────────────

    def execute(self, userdata):
        if hasattr(self.node, '_fsm_state_pub'):
            self.node._fsm_state_pub.publish(String(data='RAMPA'))

        wp     = userdata.current_wp or {}
        etiket = wp.get('label', 'DIK_EGIM')
        iniyor = 'CIKIS' in etiket           # CIKIS = rampadan iniş aşaması
        hedef_hiz = self.inis_hiz if iniyor else self.tirmanis_hiz
        yon = 'iniş' if iniyor else 'tırmanış'

        # Odom başlangıcı — sensör yoksa hiç başlama
        deadline_init = time.time() + 2.0
        while time.time() < deadline_init:
            pos, _, _, _ = self._oku()
            if pos is not None:
                break
            time.sleep(0.05)
        else:
            self.node.get_logger().error('[RAMPA] /odom yok — iptal.')
            return 'failed'

        self.node.get_logger().info(
            f'[RAMPA] {etiket} ({yon}) başladı — hedef hız {hedef_hiz:.2f} m/s, '
            f'pitch eşiği {self.PITCH_ESIK_DEG:.0f}°. Nav2 baypas.'
        )

        twist         = Twist()
        dt            = 1.0 / self.CMD_HZ
        baslama       = time.time()
        timeout_s     = self._timeout(hedef_hiz)
        deadline      = baslama + timeout_s
        faz           = 'yaklasma'
        baslangic_pos = self._oku()[0]
        faz_basi      = baslangic_pos
        kayma_basi    = None
        sonuc         = None

        while rclpy.ok():
            if self.det_store.get_field('e_stop', False):
                self.node.get_logger().error('[RAMPA] E-STOP — durduruluyor.')
                sonuc = 'e_stop'
                break

            if self.det_store.get_field('manual_mod', False):
                self.node.get_logger().warn(
                    '[RAMPA] Manuel mod — RC devraldı, bekleniyor.',
                    throttle_duration_sec=2.0,
                )
                twist.linear.x = 0.0
                self._cmd_pub.publish(twist)
                time.sleep(0.1)
                # Manuel duraklama bütçeden sayılmaz (HizlanmaState ile aynı).
                deadline += 0.1
                baslama  += 0.1
                continue

            if time.time() > deadline:
                self.node.get_logger().error(
                    f'[RAMPA] {timeout_s:.0f}s timeout — iptal.'
                )
                sonuc = 'failed'
                break

            if self.saat.doldu():
                self.node.get_logger().error(
                    '[RAMPA] §6.12 koşu süresi doldu — durduruluyor.'
                )
                sonuc = 'failed'
                break

            pos, hiz, pitch, son_odom = self._oku()

            yas = time.time() - son_odom
            if yas > self.ODOM_BAYATLAMA_S:
                self.node.get_logger().error(
                    f'[RAMPA] /odom {yas:.1f}s bayat — eğimde körlemesine '
                    'sürülmez, iptal.'
                )
                sonuc = 'failed'
                break

            gidilen = self._mesafe(faz_basi, pos)
            # İlerleme koruması TOPLAM yola bakar: gidilen faz değişiminde
            # sıfırlandığı için onunla ölçmek her faz geçişinde sahte alarm verir.
            toplam = self._mesafe(baslangic_pos, pos)
            if (time.time() - baslama > self.ODOM_ILERLEME_S
                    and toplam < self.ODOM_ILERLEME_M):
                self.node.get_logger().error(
                    f'[RAMPA] {self.ODOM_ILERLEME_S:.0f}s gazda ilerleme '
                    f'{toplam:.2f}m — takılma veya donmuş odometri, iptal.'
                )
                sonuc = 'failed'
                break

            # Geri kayma: yalnız tırmanışta anlamlı (rollback_riskli pozitif
            # pitch şartı arar). Sürekliyse tahrik yetmiyordur.
            if rollback_riskli(pitch, hiz, self.PITCH_ESIK_DEG, 0.05):
                if kayma_basi is None:
                    kayma_basi = time.time()
                    self.node.get_logger().warn(
                        f'[RAMPA] GERİ KAYMA (pitch={pitch:.1f}°, '
                        f'hız={hiz:.2f} m/s) — anti_rollback devrede olmalı.'
                    )
                elif time.time() - kayma_basi > self.GERI_KAYMA_S:
                    self.node.get_logger().error(
                        f'[RAMPA] Geri kayma {self.GERI_KAYMA_S:.1f}s sürdü — '
                        'tahrik yetersiz, gaz kesiliyor. Aracı fren tutmalı.'
                    )
                    sonuc = 'failed'
                    break
            else:
                kayma_basi = None

            yeni_faz = rampa_faz_gecisi(
                faz, abs(pitch), self.PITCH_ESIK_DEG, gidilen,
                self.YAKLASMA_MAX_M, self.EGIM_MAX_M,
            )
            if yeni_faz != faz:
                self.node.get_logger().info(
                    f'[RAMPA] faz {faz} → {yeni_faz} '
                    f'(pitch={pitch:.1f}°, {gidilen:.2f}m)'
                )
                faz      = yeni_faz
                faz_basi = pos

            if faz == 'bitti':
                break
            if faz == 'bulunamadi':
                self.node.get_logger().error(
                    f'[RAMPA] {self.YAKLASMA_MAX_M:.0f}m sürüldü, eğim '
                    f'başlamadı (pitch={pitch:.1f}°) — araç rampanın önünde '
                    'değil ya da IMU pitch okuması yanlış. İptal.'
                )
                sonuc = 'failed'
                break
            if faz == 'asildi':
                self.node.get_logger().error(
                    f'[RAMPA] Eğim {self.EGIM_MAX_M:.0f}m sürdü ve bitmedi — '
                    'pitch okuması şüpheli. İptal.'
                )
                sonuc = 'failed'
                break

            twist.linear.x  = hedef_hiz
            twist.angular.z = 0.0        # koridor düz, direksiyon kararı yok
            self._cmd_pub.publish(twist)

            self.node.get_logger().info(
                f'[RAMPA] {yon} faz={faz} pitch={pitch:.1f}° '
                f'{gidilen:.2f}m (toplam {toplam:.2f}m) hız={hiz:.2f}',
                throttle_duration_sec=0.5,
            )
            time.sleep(dt)

        # Hangi yoldan çıkılırsa çıkılsın gaz kesilir.
        twist.linear.x  = 0.0
        twist.angular.z = 0.0
        self._cmd_pub.publish(twist)

        if sonuc is not None:
            return sonuc

        # §6.10 zorunlu duruş — araç gerçekten durduktan SONRA sayılır.
        if self._dur_ve_bekle(RAMP_STOP_DURATION, etiket) == 'e_stop':
            return 'e_stop'

        self.node.get_logger().info(f'[RAMPA] {etiket} tamamlandı.')
        return 'completed'

# ═══════════════════════════════════════════════════════════════════════════════
# STATE 7 — MISSION_COMPLETE
# ═══════════════════════════════════════════════════════════════════════════════

