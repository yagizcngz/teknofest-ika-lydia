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

from rclpy.node import Node

from std_msgs.msg import Float32


from teknofest_ika.otonomi.topics import (
    MISYON_KALAN_SURE_TOPIC, KOSU_SURESI_S,
)
from teknofest_ika.otonomi.pure_logic import (
    kosu_butcesi,
)

# §6.10: dik eğim çıkış/iniş noktalarında STOP tabelası kaçırılsa bile
# zorunlu 2s bekleme uygulanması gereken waypoint etiketleri.
DIK_EGIM_ETIKETLERI = ('DIK_EGIM_GIRIS', 'DIK_EGIM_CIKIS', 'DIK_EGIM')

# §6.9 koridor ortalamasının izlendiği aşama.
ENGEBELI_ETIKET = 'ENGEBELİ_ARAZİ'




class MisyonSaati:
    """
    Şartname §6.12 koşu saati: atış dahil tüm parkur 15 dakika içinde
    tamamlanmalıdır.

    Saat hakem kronometresini taklit eder, yani MANUEL MODDA DA İŞLER. FSM'in
    kendi bekleme döngüleri (manuel duraklama, STOP bekleme) kendi bütçelerini
    uzatabilir; koşu saatinin uzatılabilir bir karşılığı sahada yoktur.

    Aşama timeout'u tek başına yetmiyordu: 11 aşama × 120 s = 22 dakikalık bir
    bütçe, koşu limitinin bir buçuk katı. Aşama timeout'ları `kalan()` ile
    kırpılmadığında araç, hakem parkurdan çıkarttığı için hiç puan getirmeyecek
    bir aşamanın içinde dakikalarca bekleyebiliyordu.
    """

    def __init__(self, node: Node, sure_s: float = KOSU_SURESI_S):
        self.node    = node
        self.sure_s  = sure_s
        self._t0     = None      # None = koşu henüz başlamadı
        self._uyarildi = set()
        self._pub    = node.create_publisher(Float32, MISYON_KALAN_SURE_TOPIC, 10)

    def baslat(self) -> None:
        self._t0 = time.time()
        self._uyarildi.clear()
        self.node.get_logger().info(
            f'[SAAT] Koşu saati başladı — §6.12 limiti {self.sure_s / 60.0:.0f} dk.'
        )
        self.yayinla()

    def basladi_mi(self) -> bool:
        return self._t0 is not None

    def kalan(self) -> float:
        """Kalan saniye. Saat başlamadıysa tüm süre kalmış sayılır."""
        if self._t0 is None:
            return self.sure_s
        return max(0.0, self.sure_s - (time.time() - self._t0))

    def doldu(self) -> bool:
        return self.basladi_mi() and self.kalan() <= 0.0

    def yayinla(self) -> None:
        self._pub.publish(Float32(data=float(self.kalan())))

    def butce(self, istenen: float) -> float:
        """
        Bir aşamaya verilebilecek gerçek süre — hesap pure_logic'te,
        test_birim.py oradaki fonksiyonu doğrudan sürer.
        """
        return kosu_butcesi(istenen, self.kalan())

    # Kalan süre uyarı eşikleri (saniye), azalan sırada.
    ESIKLER = (300.0, 120.0, 60.0)

    def rapor(self) -> None:
        """Eşik geçildikçe bir kez uyarır; her çağrıda kalan süreyi yayınlar."""
        self.yayinla()
        if not self.basladi_mi():
            return
        kalan   = self.kalan()
        gecilen = [e for e in self.ESIKLER if kalan <= e]
        if not gecilen:
            return
        # İki rapor arasında birden fazla eşik atlanmış olabilir (uzun süren
        # tek bir aşama). Yalnız en acil eşik duyurulur; atlanan üst eşikler
        # işaretlenir ki geriye dönük uyarı yağmuru olmasın.
        esik = min(gecilen)
        if esik in self._uyarildi:
            return
        self._uyarildi.update(gecilen)
        self.node.get_logger().warn(
            f'[SAAT] Koşu süresinin son {esik / 60.0:.0f} dakikası '
            f'(kalan {kalan:.0f} s).'
        )


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 1 — IDLE
# ═══════════════════════════════════════════════════════════════════════════════

