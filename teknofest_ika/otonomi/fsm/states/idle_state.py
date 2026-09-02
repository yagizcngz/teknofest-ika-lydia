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

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String

import smach

from teknofest_ika.otonomi.topics import (
    MISSION_START_TOPIC, MISYON_AKTIF_TOPIC,
)

# §6.10: dik eğim çıkış/iniş noktalarında STOP tabelası kaçırılsa bile
# zorunlu 2s bekleme uygulanması gereken waypoint etiketleri.
DIK_EGIM_ETIKETLERI = ('DIK_EGIM_GIRIS', 'DIK_EGIM_CIKIS', 'DIK_EGIM')

# §6.9 koridor ortalamasının izlendiği aşama.
ENGEBELI_ETIKET = 'ENGEBELİ_ARAZİ'




class IdleState(smach.State):
    """
    Başlangıç durumu. /mission_start → True gelince çıkar.
    Araç güçlenince hemen hareket etmemesi için bu bekleme zorunludur.

    Geçişler: 'started' → NavigateState
    """

    def __init__(self, node: Node, saat: 'MisyonSaati',
                 baslangic_bekleme: float = 3.0):
        smach.State.__init__(self, outcomes=['started'])
        self.node = node
        self.saat = saat
        self._start_received = False
        self._baslangic_bekleme = baslangic_bekleme

        self.node.create_subscription(
            Bool, MISSION_START_TOPIC, self._on_start, 10
        )
        # Veri paketi kaydedici için misyon aktif yayıncısı
        self._misyon_pub = self.node.create_publisher(Bool, MISYON_AKTIF_TOPIC, 10)
        self.node.get_logger().info('[IDLE] Hazır. /mission_start bekleniyor...')

    def _on_start(self, msg: Bool):
        if msg.data:
            self._start_received = True

    def execute(self, userdata):
        self._start_received = False
        if hasattr(self.node, '_fsm_state_pub'):
            self.node._fsm_state_pub.publish(String(data='IDLE'))
        rate = self.node.create_rate(10)
        while rclpy.ok() and not self._start_received:
            rate.sleep()
        # Saat /mission_start ile başlar, başlangıç beklemesinden ÖNCE:
        # o 3 saniye de hakem kronometresinde işliyor (§6.12).
        self.saat.baslat()
        if self._baslangic_bekleme > 0:
            self.node.get_logger().info(
                f'[IDLE] Başlangıç bekleme: {self._baslangic_bekleme:.0f}s '
                f'(SLAM + Nav2 stabilizasyonu)...'
            )
            time.sleep(self._baslangic_bekleme)
        self.node.get_logger().info('[IDLE] Görev başlıyor.')
        self._misyon_pub.publish(Bool(data=True))
        return 'started'


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 2 — NAVIGATE
# ═══════════════════════════════════════════════════════════════════════════════

