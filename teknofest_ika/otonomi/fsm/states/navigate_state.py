from teknofest_ika.otonomi.fsm.helpers.nav2_client import Nav2Client
from teknofest_ika.otonomi.fsm.helpers.misyon_saati import MisyonSaati
from teknofest_ika.otonomi.fsm.helpers.kayan_hedef_surucusu import KayanHedefSurucusu
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

from std_msgs.msg import String, UInt8

import smach

from teknofest_ika.otonomi.topics import (
    MISYON_WP_INDEX_TOPIC, RAMP_STOP_DURATION,
    PAS_HAKKI, PAS_GECILEMEZ,
)
from teknofest_ika.otonomi.pure_logic import (
    DetectionsStore, stop_check as _stop_check_pure, pas_verilebilir as _pas_verilebilir_pure,
)

# §6.10: dik eğim çıkış/iniş noktalarında STOP tabelası kaçırılsa bile
# zorunlu 2s bekleme uygulanması gereken waypoint etiketleri.
DIK_EGIM_ETIKETLERI = ('DIK_EGIM_GIRIS', 'DIK_EGIM_CIKIS', 'DIK_EGIM')

# §6.9 koridor ortalamasının izlendiği aşama.
ENGEBELI_ETIKET = 'ENGEBELİ_ARAZİ'



from teknofest_ika.otonomi.fsm.helpers.koridor_izleyici import KoridorIzleyici

class NavigateState(smach.State):
    """
    Waypoint listesini sırayla işler.

    Aşamayı sürmenin iki yolu var, seçim `hedefleme_modu` ile koşu başında
    yapılır (bkz. main):
      'harita' → waypoint'in map çerçevesindeki koordinatı Nav2'ye verilir,
                 varış Nav2'nin hedef denetleyicisinden gelir.
      'kayan'  → KayanHedefSurucusu: hedef her döngüde taramadan üretilir,
                 aşama `mesafe_m` kadar yol kat edilince biter. Waypoint
                 koordinatı da SLAM haritası da kullanılmaz.
    STOP (§6.10), pas hakkı (§9), aşama bütçesi ve E-STOP mantığı iki yolda
    da aynıdır — yalnız sürüş çağrısı değişir.

    Her waypoint için:
      1. Nav2'ye hedef gönder, bekle.
      2. Varışta waypoint etiketine göre özel mantık uygula:
           DIK_EGIM   → 2 saniye dur (şartname gereği)
           KAYAR_ENGEL → kayar_yon netleşene kadar bekle (max 10s)
           ATIS        → SHOOT_APPROACH state'ine geç
      3. Sonraki waypointı yükle.

    Geçişler:
      'next_waypoint'    → NavigateState (döngü)
      'shoot_waypoint'   → ShootApproachState
      'mission_complete' → MissionCompleteState
      'failed'           → ErrorRecoveryState
    """

    # Bir waypoint içinde kaç kez STOP tetiklenebilir. Tabela görüş alanında
    # kalıp araç ilerlemezse (cooldown dolar, tabela hâlâ görünür) dur-kalk
    # döngüsü sonsuza gider; sınıra ulaşınca STOP tespiti bu waypoint için
    # devre dışı bırakılır.
    MAX_STOP = 3

    def __init__(self, node: Node, nav: Nav2Client, det_store: DetectionsStore,
                 saat: 'MisyonSaati', stage_timeout: float = 120.0,
                 kayan: 'KayanHedefSurucusu' = None):
        smach.State.__init__(
            self,
            outcomes=['next_waypoint', 'shoot_waypoint', 'hizlanma_waypoint',
                      'mission_complete', 'sure_doldu', 'failed', 'e_stop',
                      'rampa_waypoint'],
            input_keys=['waypoints', 'wp_index'],
            output_keys=['wp_index', 'current_wp', 'retry_count']
        )
        self.node                  = node
        self.nav                   = nav
        self.det_store             = det_store
        self.saat                  = saat
        self.stage_timeout         = stage_timeout
        # None → 'harita' yolu (waypoint koordinatları Nav2'ye verilir).
        # Dolu → 'kayan' yolu: hedef her döngüde taramadan üretilir, waypoint
        # koordinatlarına ve `map` çerçevesinin doğruluğuna ihtiyaç kalmaz.
        self.kayan                 = kayan
        # §9: pas hakkı koşu başına, aşama başına değil. Sayaç state örneği
        # üzerinde tutulur; NavigateState her waypoint için yeniden execute
        # edilir ama nesne aynı kalır.
        self._pas_kullanildi       = 0
        self._wp_index_pub         = node.create_publisher(UInt8, MISYON_WP_INDEX_TOPIC, 10)
        self._stop_cooldown_bitis  = 0.0   # §6.10: son STOP sonrası tekrar tetiklenme engeli
        # §6.9 koridor ortalaması — yalnız ölçer, direksiyona karışmaz.
        self._koridor              = KoridorIzleyici(node)

    def _stop_check(self) -> bool:
        """Nav2Client.go_to() callback'i: True dönünce goal iptal edilir (§6.10 STOP)."""
        return _stop_check_pure(
            self.det_store.get_field('stop_var', False),
            self._stop_cooldown_bitis,
            time.time(),
        )

    def _pas_verilebilir(self, wp: dict, label: str) -> bool:
        """
        §9 pas kapısı. Karar pure_logic.pas_verilebilir'de; burada yalnız
        reddin gerekçesi log'a yazılır.
        """
        izin, gerekce = _pas_verilebilir_pure(
            wp.get('pas_gecilir', False), label,
            self._pas_kullanildi, PAS_HAKKI, PAS_GECILEMEZ,
        )
        if gerekce == 'sartname_yasak':
            self.node.get_logger().error(
                f'[NAVIGATE] {label} §9 gereği pas geçilemez — '
                'pas_gecilir=True yok sayılıyor.'
            )
        elif gerekce == 'hak_bitti':
            self.node.get_logger().error(
                f'[NAVIGATE] Pas hakkı bitti ({PAS_HAKKI}/koşu) — '
                f'{label} atlanmıyor, hata kurtarmaya düşülüyor.'
            )
        return izin

    def execute(self, userdata):
        if hasattr(self.node, '_fsm_state_pub'):
            self.node._fsm_state_pub.publish(String(data='NAVIGATE'))

        if self.det_store.get_field('e_stop', False):
            self.node.get_logger().error('[NAVIGATE] E-STOP aktif — navigasyon iptal.')
            return 'e_stop'

        self.saat.rapor()
        if self.saat.doldu():
            self.node.get_logger().error(
                '[NAVIGATE] §6.12 koşu süresi doldu — yeni aşamaya başlanmıyor.'
            )
            return 'sure_doldu'

        waypoints = userdata.waypoints
        idx       = userdata.wp_index

        if self.det_store.get_field('manual_mod', False):
            self.node.get_logger().warn('[NAVIGATE] Manuel mod aktif — navigasyon bekleniyor.')
            while rclpy.ok() and self.det_store.get_field('manual_mod', False):
                if self.det_store.get_field('e_stop', False):
                    self.node.get_logger().error('[NAVIGATE] E-STOP — manuel mod bekleme iptal.')
                    return 'e_stop'
                time.sleep(0.2)
            self.node.get_logger().info('[NAVIGATE] Tam otonom moda geri dönüldü.')

        if idx >= len(waypoints):
            self.node.get_logger().info('[NAVIGATE] Tüm waypointler tamamlandı.')
            return 'mission_complete'

        wp    = waypoints[idx]
        label = wp.get('label', '')

        if self.kayan is not None:
            self.node.get_logger().info(
                f'[NAVIGATE] {idx+1}/{len(waypoints)} → {label} '
                f'(kayan hedef, {float(wp.get("mesafe_m", 0.0)):.1f} m)'
            )
        else:
            self.node.get_logger().info(
                f'[NAVIGATE] {idx+1}/{len(waypoints)} → {label} '
                f'({wp["x"]:.2f}, {wp["y"]:.2f})'
            )

        # Nav2'ye git — STOP işareti görülürse ortada kesilir, 2s beklenir, tekrar gönderilir.
        # Aşama timeout'u waypoint'in TAMAMINI kapsar: her STOP'tan sonra go_to'yu
        # taze stage_timeout ile çağırmak toplam süreyi sınırsız bırakıyordu.
        stop_uygulandi = False
        stop_sayaci    = 0
        # Aşamanın kendi süresi varsa o kullanılır; yoksa ortak tavana düşülür.
        # Tek bir ortak sayı (120 s) her aşamaya aynı süreyi veriyordu: 2,5 m'lik
        # rampa inişi de, 22 m'lik sol U dönüşü de. Süreler waypoints.yaml'da
        # CAD'den ölçülen istasyonlar arası mesafeden türetildi (mesafe ÷ zemine
        # göre ortalama hız); türetimin tamamı her aşamanın yanında yazılı.
        istenen = float(wp.get('timeout_saniye', self.stage_timeout))
        # Aşama bütçesi koşuda kalan süreden uzun olamaz (§6.12). Kırpılmazsa
        # araç, hakem çoktan parkurdan çıkarttığı için puan getirmeyecek bir
        # aşamanın içinde dakikalarca bekler.
        asama_butcesi  = self.saat.butce(istenen)
        if asama_butcesi < istenen:
            self.node.get_logger().warn(
                f'[NAVIGATE] {label}: aşama bütçesi koşu saatiyle '
                f'{istenen:.0f}s → {asama_butcesi:.0f}s kırpıldı.'
            )
        wp_deadline    = time.time() + asama_butcesi

        # §6.9: tümsekli bölümde ortalanma izlenir. go_to() bloklayıcı, ölçüm
        # spin thread'indeki tarama callback'inde birikir.
        izleniyor = (label == ENGEBELI_ETIKET)
        if izleniyor:
            self._koridor.basla()

        try:
            while True:
                kalan = wp_deadline - time.time()
                dur_kontrol = (self._stop_check
                               if stop_sayaci < self.MAX_STOP else None)
                if kalan <= 0.0:
                    result = 'timeout'
                elif self.kayan is not None:
                    result = self.kayan.sur(
                        label,
                        float(wp.get('mesafe_m', 0.0)),
                        wp.get('viraj'),
                        deadline=time.time() + kalan,
                        dur_kontrol=dur_kontrol,
                        e_stop_fn=lambda: self.det_store.get_field('e_stop', False),
                    )
                else:
                    result = self.nav.go_to(
                        wp['x'], wp['y'], wp.get('yaw', 0.0),
                        timeout_sec=kalan,
                        stop_check_fn=dur_kontrol,
                    )

                # E-STOP kurtarmaya UĞRAMAZ (bkz. ErrorRecoveryState): kurtarma
                # denemesi harcamak aracı saniyelerce oyalıyordu.
                if result == 'e_stop':
                    return 'e_stop'

                if result == 'stop_requested':
                    stop_sayaci += 1
                    self.node.get_logger().info(
                        f'[NAVIGATE] §6.10 STOP işareti ({stop_sayaci}/{self.MAX_STOP}) '
                        '— araç durduruluyor, 2s bekleniyor.'
                    )
                    # Cooldown: aynı STOP işaretinin hemen tekrar tetiklenmesini engeller
                    # 3s: rampada üst-STOP ile alt-STOP arası için yeterli
                    self._stop_cooldown_bitis = time.time() + 3.0
                    self.det_store.update_field('stop_var', False)
                    stop_uygulandi = True
                    # Zorunlu bekleme navigasyon süresinden sayılmaz.
                    wp_deadline += RAMP_STOP_DURATION
                    deadline_stop = time.time() + RAMP_STOP_DURATION
                    while time.time() < deadline_stop:
                        if self.det_store.get_field('e_stop', False):
                            self.node.get_logger().error('[NAVIGATE] E-STOP — STOP bekleme iptal.')
                            return 'e_stop'
                        time.sleep(0.1)
                    self.node.get_logger().info('[NAVIGATE] §6.10 STOP bekleme tamam — devam ediliyor.')
                    if stop_sayaci >= self.MAX_STOP:
                        self.node.get_logger().warn(
                            f'[NAVIGATE] {label}: STOP sınırı ({self.MAX_STOP}) doldu — '
                            'bu waypoint için STOP tespiti devre dışı, dur-kalk döngüsü kesildi.'
                        )
                    continue

                if result == 'success':
                    break

                # 'failed' veya 'timeout'
                if self._pas_verilebilir(wp, label):
                    self._pas_kullanildi += 1
                    bedel = float(wp.get('pas_puan', 0.0))
                    self.node.get_logger().warn(
                        f'[NAVIGATE] {label} başarısız — pas geçiliyor '
                        f'({self._pas_kullanildi}/{PAS_HAKKI} hak). '
                        f'§9 bedeli: -{bedel:.0f} puan. Araç bir sonraki hedefe '
                        'sürüyor; geçerli bir pas için aşama sonuna ELLE '
                        'taşınması gerekir.'
                    )
                    userdata.wp_index  = idx + 1
                    userdata.current_wp = wp
                    return 'next_waypoint'
                else:
                    self.node.get_logger().warn(
                        f'[NAVIGATE] {label} başarısız — hata kurtarma.'
                    )
                    return 'failed'

            # ── Varış sonrası özel mantık ─────────────────────────────────

        finally:
            # Hangi yoldan çıkılırsa çıkılsın izleme kapanmalı: açık
            # kalırsa sonraki aşamanın taramaları da §6.9 ölçümüne
            # yazılır ve özet yanlış aşamayı anlatır.
            if izleniyor:
                self._koridor.bitir()

        # §6.10 yedek tetikleyici: dik eğim giriş/çıkış waypoint'ine
        # YOLO'nun STOP tabelasını kaçırması nedeniyle hiç STOP
        # tetiklenmeden varılmışsa, zorunlu 2s bekleme burada mesafe/
        # konum tabanlı olarak (waypoint varışı = tetikleyici) uygulanır.
        # Tek bir görüntü tespitine bağımlılığı ortadan kaldırır.
        if (label in DIK_EGIM_ETIKETLERI and not stop_uygulandi
                and wp.get('type') != 'rampa'):
            self.node.get_logger().warn(
                f'[NAVIGATE] {label}: STOP tabelası tespit edilmeden hedefe '
                'varıldı — §6.10 yedek (konum tabanlı) 2s bekleme uygulanıyor.'
            )
            deadline_stop = time.time() + RAMP_STOP_DURATION
            while time.time() < deadline_stop:
                if self.det_store.get_field('e_stop', False):
                    self.node.get_logger().error(
                        '[NAVIGATE] E-STOP — yedek STOP bekleme iptal.'
                    )
                    return 'e_stop'
                time.sleep(0.1)
            self.node.get_logger().info(
                '[NAVIGATE] §6.10 yedek STOP bekleme tamam — devam ediliyor.'
            )

        if label == 'KAYAR_ENGEL':
            # Kayar engelin hangi tarafa geçtiğini bekle (max 10s)
            self.node.get_logger().info(
                '[NAVIGATE] KAYAR_ENGEL: geçiş yönü bekleniyor...'
            )
            deadline = time.time() + 10.0
            while time.time() < deadline:
                if self.det_store.get_field('e_stop', False):
                    self.node.get_logger().error('[NAVIGATE] E-STOP — KAYAR_ENGEL bekleme iptal.')
                    return 'e_stop'
                yon = self.det_store.get_field('kayar_yon', 'bilinmiyor')
                if yon != 'bilinmiyor':
                    self.node.get_logger().info(
                        f'[NAVIGATE] Kayar engel → {yon} tarafa geçti.'
                    )
                    break
                time.sleep(0.2)
            else:
                self.node.get_logger().warn(
                    '[NAVIGATE] Kayar engel yönü alınamadı (timeout) — devam ediliyor.'
                )

        # İndeksi artır ve GCS'e bildir
        userdata.wp_index  = idx + 1
        userdata.current_wp = wp
        # Bir aşama tamamlandığı an kurtarma bütçesi tazelenir; sayaç görev
        # boyunca birikirse parkurun farklı yerlerindeki üç bağımsız hata
        # görevi iptal ettirir.
        userdata.retry_count = 0
        self._wp_index_pub.publish(UInt8(data=min(idx + 1, 255)))

        if wp.get('type') == 'shoot':
            return 'shoot_waypoint'

        if wp.get('type') == 'hizlanma':
            return 'hizlanma_waypoint'

        if wp.get('type') == 'rampa':
            return 'rampa_waypoint'

        return 'next_waypoint'


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 3 — SHOOT_APPROACH
# ═══════════════════════════════════════════════════════════════════════════════

