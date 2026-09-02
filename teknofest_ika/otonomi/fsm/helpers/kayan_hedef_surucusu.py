from teknofest_ika.otonomi.fsm.helpers.nav2_client import Nav2Client
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

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


from teknofest_ika.otonomi.topics import (
    EKF_ODOM_TOPIC, SCAN_FILTERED_TOPIC,
    KORIDOR_GENISLIGI_M, KAYAN_HEDEF_PERIYOT_S, KAYAN_HEDEF_YOK_SINIR,
    KAYAN_ODOM_BAYATLAMA_S, LIDAR_MONTAJ_YAW_RAD,
)
from teknofest_ika.otonomi.pure_logic import (
    koridor_merkez_cizgisi, kayan_hedef, ic_duvar_hedefi,
    kayan_hedef_karari, quat_yaw, arac_hedefini_odoma_tasi,
)

# §6.10: dik eğim çıkış/iniş noktalarında STOP tabelası kaçırılsa bile
# zorunlu 2s bekleme uygulanması gereken waypoint etiketleri.
DIK_EGIM_ETIKETLERI = ('DIK_EGIM_GIRIS', 'DIK_EGIM_CIKIS', 'DIK_EGIM')

# §6.9 koridor ortalamasının izlendiği aşama.
ENGEBELI_ETIKET = 'ENGEBELİ_ARAZİ'




class KayanHedefSurucusu:
    """
    Haritasız sürüş: hedef her döngüde LiDAR taramasından yeniden üretilir.

    NEDEN
    ─────────────────────────────────────────────────────────────────────────
    'harita' yolunda hedefler `map` çerçevesinde sabit noktalardır ve o
    çerçevenin orijini aracın boot ettiği yerdir. Harita koşarken kurulduğu
    için bir istasyonun map koordinatı ÖNCEDEN bilinemez; ancak parkur CAD'i
    ile harita arasında ölçülecek tek bir katı dönüşümle bağlanabilir
    (`waypoints.yaml` → `parkur_cad.donusum`). O dönüşüm ölçülmemişse ya da
    SLAM'in `map→odom` düzeltmesi koşu ortasında sıçrarsa hedefler kayar.

    Bu sınıf o zincire hiç girmez. Hedef araç çerçevesinde doğar, Nav2'ye
    `base_link` çerçevesinde verilir ve bir sonraki döngüde yenisiyle
    değiştirilir. Odometri yalnız "bu aşamada ne kadar yol gittim" sorusu
    için kullanılır; hedefin kendisine HİÇ karışmaz, dolayısıyla odometri
    kayması hedefe birikmez.

    HEDEF NEREDEN ÜRETİLİR
    ─────────────────────────────────────────────────────────────────────────
    Önce koridorun merkez çizgisi denenir (`koridor_merkez_cizgisi` +
    `kayan_hedef`). Düz şeritte çalışan yol budur. U dönüşünde 180° sensör
    menzilinin içinde tamamlandığı için merkez çizgisi 5–13 m boyunca hedef
    üretemiyor; orada aşamanın `viraj` alanında yazan iç duvar takip edilir
    (`ic_duvar_hedefi`). Seçim kendiliğinden olur: merkez çizgisi hedef
    veremediği anda iç duvara düşülür, dönüşün nerede başladığını bilmek
    gerekmez.

    Aşama, waypoint koordinatına varışla değil, `mesafe_m` kadar YOL
    KAT EDİLMESİYLE biter. Mesafeler parkur CAD'inden planlayıcıyla ölçüldü
    (`scripts/parkur_cad/donus.py`), kuş uçuşu değil gerçek yol uzunluğudur.
    """

    # Odometri ilerlemiyorsa (enkoder takılı değil / kopuk) bu yalnız
    # UYARILIR, aşama düşürülmez: §6.8 kayar engelde araç meşru olarak
    # bekliyor olabilir. Gerçek takılmayı aşamanın kendi bütçesi yakalar.
    ODOM_ILERLEME_UYARI_S = 6.0
    ODOM_ILERLEME_M       = 0.20

    def __init__(self, node: Node, nav: Nav2Client, lidar_yaw_rad: float = LIDAR_MONTAJ_YAW_RAD):
        self.node = node
        self.nav  = nav
        # LiDAR gövdeye dönük monte; düzeltme TF'te uygulanıyor, ham taramada
        # değil (KoridorIzleyici ile aynı gerekçe).
        self._lidar_yaw = lidar_yaw_rad

        self._lock       = threading.Lock()
        self._scan       = None    # (ranges, angle_min, angle_increment)
        self._konum      = None    # son EKF konumu
        self._poz        = None    # (x, y, yaw) — hedefi odom'a taşımak için
        self._odom_zaman = 0.0     # wall-clock — son EKF mesajının geliş anı
        self._yol        = 0.0     # aşama başından biriken YOL UZUNLUĞU [m]

        node.create_subscription(LaserScan, SCAN_FILTERED_TOPIC, self._on_scan, 10)
        # EKF çıkışı, ham /odom DEĞİL. /odom seri_kopru'nun tekerlek
        # odometrisidir ve arka aks tek parça olduğu için tek enkoderle
        # koşuyor: `_odometri()` orada d_theta = 0 bırakır, yani /odom'un
        # yaw'ı HEP SIFIRDIR. Hedefi o yaw ile odom'a taşımak, araç ilk
        # dönüşten sonra hedefleri sabit bir yöne koymak demekti.
        # Yön IMU'nun işi ve IMU yalnız EKF'te füzyona giriyor;
        # /odometry/filtered ayrıca Nav2'nin ve odom→base_footprint TF'inin
        # kullandığı pozla aynı, yani hedef Nav2'nin gördüğü çerçeveye oturur.
        # (HizlanmaState ve RampaState bilerek ham /odom'da kalıyor: onlar
        # yalnız mesafe/hız okuyor ve kopuk enkoderi ham veride görmek
        # istiyorlar.)
        node.create_subscription(Odometry, EKF_ODOM_TOPIC, self._on_odom, 10)

    def _on_scan(self, msg: LaserScan):
        with self._lock:
            self._scan = (list(msg.ranges), msg.angle_min, msg.angle_increment)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self._lock:
            if self._konum is not None:
                # Kuş uçuşu değil, artımların toplamı: U dönüşünde ikisi
                # 22 m'ye karşı 11 m ayrışır ve kuş uçuşu ölçüt aşamayı
                # yolun yarısında bitirirdi.
                self._yol += math.hypot(p.x - self._konum[0], p.y - self._konum[1])
            self._konum      = (p.x, p.y)
            self._poz        = (p.x, p.y, quat_yaw(q.x, q.y, q.z, q.w))
            self._odom_zaman = time.time()

    def hedef_uret(self, taraf):
        """Taramadan (x, y, yaw) üretir. Dönüş: (hedef | None, kaynak)."""
        with self._lock:
            scan = self._scan
        if scan is None:
            return None, 'tarama_yok'
        ranges, amin, inc = scan

        cizgi = koridor_merkez_cizgisi(ranges, amin, inc, self._lidar_yaw,
                                       KORIDOR_GENISLIGI_M)
        hedef = kayan_hedef(cizgi)
        if hedef is not None:
            return hedef, 'merkez'

        if taraf:
            hedef = ic_duvar_hedefi(ranges, amin, inc, self._lidar_yaw, taraf)
            if hedef is not None:
                return hedef, f'ic_duvar_{taraf}'

        return None, 'hedef_yok'

    def sur(self, label: str, mesafe_m: float, taraf, deadline: float,
            dur_kontrol=None, e_stop_fn=None) -> str:
        """
        Aşamayı kayan hedefle sürer. Bloklar.
        Dönüş: 'success' | 'timeout' | 'failed' | 'stop_requested' | 'e_stop'
        """
        if mesafe_m <= 0.0:
            self.node.get_logger().error(
                f'[KAYAN] {label}: mesafe_m yok — aşamanın nerede biteceği '
                'bilinmiyor, sürülmüyor.'
            )
            return 'failed'

        with self._lock:
            self._yol   = 0.0
            self._konum = None

        self.node.get_logger().info(
            f'[KAYAN] {label} → {mesafe_m:.1f} m yol, '
            f'viraj={taraf or "yok"}, bütçe={deadline - time.time():.0f}s'
        )

        hedefsiz       = 0
        sonraki_hedef  = 0.0
        son_ilerleme_t = time.time()
        son_ilerleme_y = 0.0
        try:
            while rclpy.ok():
                simdi = time.time()
                if e_stop_fn and e_stop_fn():
                    self.node.get_logger().error('[KAYAN] E-STOP — sürüş iptal.')
                    return 'e_stop'
                if dur_kontrol and dur_kontrol():
                    return 'stop_requested'
                if simdi >= deadline:
                    with self._lock:
                        yol = self._yol
                    self.node.get_logger().warn(
                        f'[KAYAN] {label}: bütçe doldu — {yol:.1f}/{mesafe_m:.1f} m.'
                    )
                    return 'timeout'

                with self._lock:
                    yol        = self._yol
                    odom_zaman = self._odom_zaman

                if odom_zaman == 0.0 or simdi - odom_zaman > KAYAN_ODOM_BAYATLAMA_S:
                    # Kat edilen yol ölçülemiyorsa aşamanın biteceği nokta da
                    # bilinmiyor demektir; kör sürmek yerine kurtarmaya düşülür.
                    self.node.get_logger().error(
                        f'[KAYAN] {label}: /odometry/filtered yok ya da bayat — '
                        'kat edilen '
                        'yol ölçülemiyor, sürüş kesiliyor.'
                    )
                    return 'failed'

                karar = kayan_hedef_karari(yol, mesafe_m, hedefsiz,
                                           KAYAN_HEDEF_YOK_SINIR)
                if karar == 'tamam':
                    self.node.get_logger().info(
                        f'[KAYAN] {label} tamam — {yol:.1f} m kat edildi.'
                    )
                    return 'success'
                if karar == 'hedef_yok':
                    self.node.get_logger().error(
                        f'[KAYAN] {label}: {KAYAN_HEDEF_YOK_SINIR} ardışık '
                        f'döngüde hedef üretilemedi ({yol:.1f}/{mesafe_m:.1f} m) — '
                        'koridor taramada tutmuyor.'
                    )
                    return 'failed'

                if yol - son_ilerleme_y >= self.ODOM_ILERLEME_M:
                    son_ilerleme_y = yol
                    son_ilerleme_t = simdi
                elif simdi - son_ilerleme_t > self.ODOM_ILERLEME_UYARI_S:
                    self.node.get_logger().warn(
                        f'[KAYAN] {label}: {self.ODOM_ILERLEME_UYARI_S:.0f}s '
                        f'ilerleme yok ({yol:.1f} m) — enkoder ya da engel?'
                    )
                    son_ilerleme_t = simdi

                if simdi >= sonraki_hedef:
                    hedef, kaynak = self.hedef_uret(taraf)
                    if hedef is None:
                        hedefsiz += 1
                        self.node.get_logger().warn(
                            f'[KAYAN] {label}: hedef üretilemedi ({kaynak}) '
                            f'{hedefsiz}/{KAYAN_HEDEF_YOK_SINIR}'
                        )
                    else:
                        hedefsiz = 0
                        with self._lock:
                            poz = self._poz
                        # Hedef araç çerçevesinde doğdu; odom'a taşınmadan
                        # gönderilirse Nav2 her yeniden planlamada onu o anki
                        # poza göre yeniden çözer ve hedef araçla kayar.
                        ox, oy, oyaw = arac_hedefini_odoma_tasi(hedef, poz)
                        if self.nav.hedef_gonder(ox, oy, oyaw):
                            self.node.get_logger().info(
                                f'[KAYAN] {label}: hedef {kaynak} '
                                f'araçtan ({hedef[0]:.2f}, {hedef[1]:.2f}) → '
                                f'odom ({ox:.2f}, {oy:.2f}) '
                                f'yol {yol:.1f}/{mesafe_m:.1f} m'
                            )
                    sonraki_hedef = simdi + KAYAN_HEDEF_PERIYOT_S

                time.sleep(0.1)
        finally:
            # Hangi yoldan çıkılırsa çıkılsın etkin hedef iptal edilmeli:
            # kalırsa Nav2 aşama bittikten sonra da ileri sürmeye devam eder.
            self.nav.iptal()

        return 'failed'


