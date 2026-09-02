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

import time
import threading

from rclpy.node import Node

from std_msgs.msg import Bool, String

import smach

from teknofest_ika.otonomi.topics import (
    SHOOT_CMD_TOPIC, SHOOT_RESULT_TOPIC,
    LASER_FIRE_DURATION,
)
from teknofest_ika.otonomi.pure_logic import (
    DetectionsStore,
)

# §6.10: dik eğim çıkış/iniş noktalarında STOP tabelası kaçırılsa bile
# zorunlu 2s bekleme uygulanması gereken waypoint etiketleri.
DIK_EGIM_ETIKETLERI = ('DIK_EGIM_GIRIS', 'DIK_EGIM_CIKIS', 'DIK_EGIM')

# §6.9 koridor ortalamasının izlendiği aşama.
ENGEBELI_ETIKET = 'ENGEBELİ_ARAZİ'




class ShootState(smach.State):
    """
    Atış yapma durumu — Şartname 6.10 uyumlu.

    Şartname kuralları:
      - En fazla 3 deneme hakkı, en iyi puan geçerli.
      - Lazer min 1 saniye hedefte kalacak (NANO sayar).
      - Lazer aktifken araç hareket etmeyecek (seri_kopru.py PKT_LAZER=1'de bloklar).

    Akış her deneme için:
      /shoot_command → True (seri_kopru PKT_LAZER=1 gönderir)
      /shoot_confirmed → True beklenir (seri_kopru PKT_LAZER=0 echo'sunda publish eder)
      Timeout → bir sonraki denemeye geç
      Onay geldiğinde kalan denemeler kullanılmaz (şartname "en fazla 3" der,
      "her zaman 3" değil; boşa harcanan her deneme 9 saniyeye mal olur).

    Geçişler: 'shot_fired' → NavigateState
              'failed'     → ErrorRecoveryState (E-STOP)
    """

    MAX_DENEME = 3       # Şartname: en fazla 3 deneme
    TIMEOUT_S  = 8.0     # Her deneme için max bekleme

    def __init__(self, node: Node, det_store: DetectionsStore,
                 saat: 'MisyonSaati'):
        smach.State.__init__(self, outcomes=['shot_fired', 'failed', 'e_stop'])
        self.saat = saat
        self.node              = node
        self.det_store         = det_store
        self._confirmed_event  = threading.Event()

        self._shoot_pub = node.create_publisher(Bool, SHOOT_CMD_TOPIC, 10)
        node.create_subscription(
            Bool, SHOOT_RESULT_TOPIC, self._on_confirmed, 10
        )

    def _on_confirmed(self, msg: Bool):
        if msg.data:
            self._confirmed_event.set()

    def execute(self, userdata):
        if hasattr(self.node, '_fsm_state_pub'):
            self.node._fsm_state_pub.publish(String(data='SHOOT'))
        basarili_deneme = 0

        for deneme in range(1, self.MAX_DENEME + 1):
            # Her denemenin BAŞINDA kontrol edilir. Kontrol yalnız döngü
            # sonunda olsaydı E-STOP aktifken bu state'e girilmesi lazeri
            # ateşlerdi.
            if self.det_store.get_field('e_stop', False):
                self.node.get_logger().error(
                    '[SHOOT] E-STOP aktif — ateş edilmiyor.'
                )
                self._shoot_pub.publish(Bool(data=False))
                return 'e_stop'

            # Koşu saati yalnız denemeler ARASINDA bakılır: başlamış bir atış
            # yarıda kesilemez, §6.10 lazerin en az 1 s aktif kalmasını ve o
            # süre boyunca araca hareket verilmemesini şart koşuyor.
            if deneme > 1 and self.saat.doldu():
                self.node.get_logger().error(
                    '[SHOOT] §6.12 koşu süresi doldu — kalan denemeler '
                    'kullanılmıyor.'
                )
                break

            self.node.get_logger().info(
                f'[SHOOT] Deneme {deneme}/{self.MAX_DENEME}'
            )
            self._confirmed_event.clear()

            # Ateş komutu — seri_kopru PKT_LAZER=1 → MEGA → NANO → Lazer
            # NOT: seri_kopru bu True yayınını aldığı an _lazer_aktif=True
            # yapar ve TÜM hareket komutlarını PKT_DUR'a çevirir (movement
            # lock). Bu kilit, aşağıda False yayınlanana kadar açık kalır.
            self._shoot_pub.publish(Bool(data=True))
            ates_baslangic = time.time()

            # Onay bekle (seri_kopru PKT_LAZER=0 echo'sunda publish eder)
            # threading.Event.wait — spin thread'den gelen set() atomik olarak yakalanır
            onaylandi = self._confirmed_event.wait(timeout=self.TIMEOUT_S)

            # Şartname §6.10: lazer aktif olduktan sonra EN AZ 1 saniye
            # (LASER_FIRE_DURATION) hedefte/aktif kalmalı. Donanım (Nano)
            # zamanlamasına tek başına güvenmek yerine yazılım seviyesinde
            # de minimum süreyi garanti ediyoruz — hareket kilidi bu süre
            # boyunca kesinlikle açık kalır.
            gecen = time.time() - ates_baslangic
            if gecen < LASER_FIRE_DURATION:
                time.sleep(LASER_FIRE_DURATION - gecen)

            # Hareket kilidini aç — bir sonraki deneme/parkura geçiş için
            # seri_kopru._lazer_aktif'i False'a çeker. Bu çağrı yapılmazsa
            # kilit kalıcı olarak açık kalır ve araç bir daha hiç hareket
            # edemez (kritik güvenlik/görev hatası).
            self._shoot_pub.publish(Bool(data=False))

            if onaylandi:
                basarili_deneme = deneme
                self.node.get_logger().info(
                    f'[SHOOT] Deneme {deneme} onaylandı ✓'
                )
                break

            self.node.get_logger().warn(
                f'[SHOOT] Deneme {deneme} onayı gelmedi (timeout).'
            )

            # Son deneme değilse kısa bekleme sonrası tekrar dene
            if deneme < self.MAX_DENEME:
                time.sleep(1.0)

        if basarili_deneme > 0:
            self.node.get_logger().info(
                f'[SHOOT] {basarili_deneme}. denemede onaylandı. Devam.'
            )
        else:
            self.node.get_logger().warn(
                '[SHOOT] Hiçbir deneme onaylanmadı — devam ediliyor.'
            )

        return 'shot_fired'


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 5 — HIZLANMA
# ═══════════════════════════════════════════════════════════════════════════════

