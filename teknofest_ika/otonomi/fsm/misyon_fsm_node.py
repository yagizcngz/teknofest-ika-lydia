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
import threading
import yaml

import rclpy

from std_msgs.msg import Bool, String, UInt8

import smach

from teknofest_ika.otonomi.topics import (
    FSM_STATE_TOPIC, DETECTIONS_TOPIC,
    MOVING_OBS_DIR_TOPIC, E_STOP_TOPIC, MOD_AKTIF_TOPIC,
    KOSU_SURESI_S, PAS_HAKKI,
)
from teknofest_ika.otonomi.pure_logic import (
    DetectionsStore, hedefleme_modu_sec,
)

# §6.10: dik eğim çıkış/iniş noktalarında STOP tabelası kaçırılsa bile
# zorunlu 2s bekleme uygulanması gereken waypoint etiketleri.
DIK_EGIM_ETIKETLERI = ('DIK_EGIM_GIRIS', 'DIK_EGIM_CIKIS', 'DIK_EGIM')

# §6.9 koridor ortalamasının izlendiği aşama.
ENGEBELI_ETIKET = 'ENGEBELİ_ARAZİ'



from teknofest_ika.otonomi.fsm.helpers.nav2_client import Nav2Client
from teknofest_ika.otonomi.fsm.helpers.misyon_saati import MisyonSaati
from teknofest_ika.otonomi.fsm.helpers.kayan_hedef_surucusu import KayanHedefSurucusu
from teknofest_ika.otonomi.fsm.states.idle_state import IdleState
from teknofest_ika.otonomi.fsm.states.navigate_state import NavigateState
from teknofest_ika.otonomi.fsm.states.shoot_approach_state import ShootApproachState
from teknofest_ika.otonomi.fsm.states.shoot_state import ShootState
from teknofest_ika.otonomi.fsm.states.hizlanma_state import HizlanmaState
from teknofest_ika.otonomi.fsm.states.rampa_state import RampaState
from teknofest_ika.otonomi.fsm.states.mission_complete_state import MissionCompleteState
from teknofest_ika.otonomi.fsm.states.error_recovery_state import ErrorRecoveryState
def load_waypoints(yaml_path: str):
    """
    waypoints.yaml → (waypoint listesi, parametreler dict).

    'ATIS_BOLGESI' isimli aşama type='shoot' olarak işaretlenir.
    'DIK_EGIM' ve 'KAYAR_ENGEL' etiketleri NavigateState'te özel işlenir.
    """
    try:
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        asamalar  = data.get('asamalar', [])
        waypoints = []
        for a in asamalar:
            wp    = a.get('waypoint', {})
            isim  = a.get('isim', '')
            if isim == 'ATIS_BOLGESI':
                tip = 'shoot'
            elif isim == 'HIZLANMA_PARKURU':
                tip = 'hizlanma'
            elif isim in DIK_EGIM_ETIKETLERI:
                # §6.10 eğimi RampaState sürüyor; Nav2 yalnız rampanın
                # başına kadar getiriyor.
                tip = 'rampa'
            else:
                tip = 'nav'
            waypoints.append({
                'x':           wp.get('x', 0.0),
                'y':           wp.get('y', 0.0),
                'yaw':         wp.get('yaw', 0.0),
                'type':        tip,
                'label':       isim,
                'pas_gecilir': a.get('pas_gecilir', False),
                # §9: pas geçilen aşamanın alınabilecek en yüksek puanı eksi
                # olarak yazılır. Log'da bedelin görünmesi için taşınıyor.
                'pas_puan':    a.get('pas_puan', 0.0),
                # Kayan hedef yolu için: aşamanın CAD'den ölçülen YOL uzunluğu
                # ve (varsa) U dönüşünde takip edilecek iç duvarın yanı.
                # 'harita' yolunda ikisi de okunmaz.
                'mesafe_m':    a.get('mesafe_m', 0.0),
                'viraj':       a.get('viraj'),
            })

        parametreler = data.get('parametreler', {})
        print(f'[WAYPOINTS] {len(waypoints)} waypoint yüklendi.')

        # waypoints.yaml deposa placeholder (hepsi 0.0) olarak giriyor. Dolmadan
        # çalıştırılırsa NavigateState her istasyon için aynı hedefi (map
        # orijini) gönderir: araç ilk hedefe varır, sonraki aşamalar anında
        # "ulaşıldı" sayılır ve FSM parkuru hiç sürmeden tamamlar. Log'da
        # tek bir hata satırı bile görünmez — o yüzden burada yüksek sesle
        # uyarılıyor.
        sifir = [w for w in waypoints if w['x'] == 0.0 and w['y'] == 0.0]
        if sifir and len(sifir) == len(waypoints):
            print('[WAYPOINTS] ' + '!' * 60)
            print('[WAYPOINTS] TÜM KOORDİNATLAR PLACEHOLDER (0,0) — parkur '
                  'sürülmez.')
            print('[WAYPOINTS] SLAM haritası alınıp rviz2 "2D Nav Goal" ile '
                  f'{yaml_path} doldurulmalı.')
            print('[WAYPOINTS] ' + '!' * 60)
        elif sifir:
            isimler = ', '.join(w['label'] for w in sifir)
            print(f'[WAYPOINTS] UYARI: {len(sifir)} waypoint hâlâ (0,0) — '
                  f'{isimler}')

        return waypoints, parametreler

    except FileNotFoundError:
        print(f'[WAYPOINTS] UYARI: {yaml_path} bulunamadı. Boş liste.')
        return [], {}
    except Exception as e:
        print(f'[WAYPOINTS] HATA: {e}')
        return [], {}


# ═══════════════════════════════════════════════════════════════════════════════
# ANA ÇALIŞMA
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = rclpy.create_node('misyon_fsm')

    # ── Detections Store ──────────────────────────────────────────────
    det_store = DetectionsStore()

    # FSM durum yayını: tüm ekiplerin izleyebileceği string durum
    fsm_state_pub = node.create_publisher(String, FSM_STATE_TOPIC, 10)
    # node üzerinden state'lerin erişmesi için referans
    node._fsm_state_pub = fsm_state_pub

    def _on_detections(msg: String):
        try:
            data = json.loads(msg.data)
            det_store.update(data)
        except (json.JSONDecodeError, ValueError):
            pass

    node.create_subscription(String, DETECTIONS_TOPIC, _on_detections, 10)

    def _on_obs_direction(msg: String):
        det_store.update_field('kayar_yon', msg.data)

    node.create_subscription(String, MOVING_OBS_DIR_TOPIC, _on_obs_direction, 10)

    def _on_e_stop(msg: Bool):
        det_store.update_field('e_stop', msg.data)
        if msg.data:
            node.get_logger().error('!!! E-STOP ALINDI — TÜM NAVIGASYON DURDURULUYOR !!!')

    node.create_subscription(Bool, E_STOP_TOPIC, _on_e_stop, 10)

    def _on_mod(msg: UInt8):
        # 0=MANUAL → FSM navigasyonu bekletir (RC kumanda sürüyor)
        # 2=FULL_AUTO → tam otonom
        det_store.update_field('manual_mod', msg.data == 0)

    node.create_subscription(UInt8, MOD_AKTIF_TOPIC, _on_mod, 10)

    # ── Nav2 Client ───────────────────────────────────────────────────
    nav = Nav2Client(node)

    # ── Waypoint yükle ────────────────────────────────────────────────
    import os
    from ament_index_python.packages import get_package_share_directory
    try:
        pkg_share = get_package_share_directory('teknofest_ika')
        wp_path   = os.path.join(pkg_share, 'config', 'waypoints.yaml')
    except Exception:
        wp_path = os.path.expanduser('~/lydia_ws/config/waypoints.yaml')

    waypoints, parametreler = load_waypoints(wp_path)

    stage_timeout      = float(parametreler.get('asama_timeout_saniye', 120.0))
    baslangic_bekleme  = float(parametreler.get('baslangic_bekleme',    3.0))
    kosu_suresi        = float(parametreler.get('kosu_suresi_saniye', KOSU_SURESI_S))
    # §6.10 rampa hızları — sahada ölçülecek, waypoints.yaml'dan canlı okunur.
    rampa_tirmanis     = float(parametreler.get('rampa_tirmanis_hiz', 0.60))
    rampa_inis         = float(parametreler.get('rampa_inis_hiz',     0.40))
    node.get_logger().info(f'Aşama timeout: {stage_timeout:.0f}s | Başlangıç bekleme: {baslangic_bekleme:.0f}s (waypoints.yaml)')

    # ── Hedefleme yolu: 'harita' | 'kayan' | 'oto' ────────────────────────
    # Varsayılan waypoints.yaml'dan gelir, `-p hedefleme_modu:=kayan` ile
    # ezilir (lydia_startup.sh HEDEFLEME_MODU değişkeni). Seçim koşu başında
    # BİR KEZ yapılır ve gerekçesiyle loglanır.
    node.declare_parameter('hedefleme_modu',
                           str(parametreler.get('hedefleme_modu', 'harita')))
    istenen_mod = node.get_parameter('hedefleme_modu').value
    waypoint_dolu = any(w['x'] != 0.0 or w['y'] != 0.0 for w in waypoints)
    hedefleme_modu, mod_gerekce = hedefleme_modu_sec(istenen_mod, waypoint_dolu)

    kayan_surucu = None
    if hedefleme_modu == 'kayan':
        eksik = [w['label'] for w in waypoints if float(w.get('mesafe_m', 0.0)) <= 0.0]
        if eksik:
            # Mesafesiz aşama kayan modda nerede biteceğini bilmez ve bütçesi
            # dolana kadar sürer. Sessizce sürmektense baştan söylenir.
            node.get_logger().error(
                f'[MOD] kayan hedef seçildi ama mesafe_m yok: {", ".join(eksik)} — '
                'bu aşamalar timeout\'a düşecek.'
            )
        kayan_surucu = KayanHedefSurucusu(node, nav)
        node.get_logger().warn(
            f'[MOD] KAYAN HEDEF ({mod_gerekce}) — hedefler LiDAR taramasından '
            'üretiliyor, waypoint koordinatları ve SLAM haritası KULLANILMIYOR. '
            'Aşamalar mesafe_m kadar yol kat edilince biter.'
        )
    else:
        node.get_logger().info(
            f'[MOD] HARİTA HEDEFİ ({mod_gerekce}) — waypoint koordinatları '
            'Nav2\'ye map çerçevesinde veriliyor.'
        )
        if not waypoint_dolu:
            node.get_logger().error(
                '[MOD] UYARI: waypoint\'ler 11/11 (0,0) — bu koşu parkuru hiç '
                'sürmeden "tamamlandı" der. hedefleme_modu:=kayan ya da :=oto '
                'ile kayan hedefe geçilebilir.'
            )

    saat = MisyonSaati(node, kosu_suresi)
    node.get_logger().info(
        f'Koşu limiti: {kosu_suresi / 60.0:.0f} dk (§6.12) | '
        f'Pas hakkı: {PAS_HAKKI}/koşu (§9) | '
        f'Rampa (Nav2 baypas): tırmanış {rampa_tirmanis:.2f} / '
        f'iniş {rampa_inis:.2f} m/s'
    )
    # Aşama bütçelerinin TOPLAMI koşu limitini aşarsa tek bir takılan aşama
    # koşunun geri kalanını yiyebilir. Toplam artık aşama başına yazılan
    # sürelerden hesaplanıyor; ortak tavan yalnız süresi olmayan aşamalar için.
    butce_toplami = sum(float(w.get('timeout_saniye', stage_timeout))
                        for w in waypoints) if waypoints else stage_timeout
    node.get_logger().info(
        f'Aşama bütçeleri toplamı: {butce_toplami:.0f}s / {kosu_suresi:.0f}s '
        f'(pay {kosu_suresi - butce_toplami:.0f}s). Atış, hızlanma ve zorunlu '
        'beklemeler bu toplamın DIŞINDA — ayrı durumlarda geçiyor.'
    )
    if butce_toplami > kosu_suresi:
        node.get_logger().warn(
            f'Aşama bütçesi toplamı ({butce_toplami:.0f}s) koşu limitini '
            f'({kosu_suresi:.0f}s) aşıyor — aşama süreleri koşu saatiyle '
            'kırpılacak.'
        )

    # ── SMACH FSM Kurulumu ────────────────────────────────────────────
    sm = smach.StateMachine(outcomes=['GOREV_TAMAMLANDI', 'GOREV_IPTAL'])

    sm.userdata.waypoints   = waypoints
    sm.userdata.wp_index    = 0
    sm.userdata.current_wp  = None
    sm.userdata.retry_count = 0

    with sm:
        smach.StateMachine.add(
            'IDLE',
            IdleState(node, saat, baslangic_bekleme),
            transitions={'started': 'NAVIGATE'}
        )

        smach.StateMachine.add(
            'NAVIGATE',
            NavigateState(node, nav, det_store, saat, stage_timeout,
                          kayan=kayan_surucu),
            transitions={
                'next_waypoint':    'NAVIGATE',
                'shoot_waypoint':   'SHOOT_APPROACH',
                'hizlanma_waypoint':'HIZLANMA',
                'mission_complete': 'MISSION_COMPLETE',
                # §6.12 süre dolduğunda görev tamamlanmış değildir: araç
                # parkurdan çıkarılır. 'COMPLETE' yayınlamak logda gerçek
                # bitişten ayırt edilemez olurdu.
                'sure_doldu':       'MISSION_ABORT',
                'failed':           'ERROR_RECOVERY',
                # E-STOP kurtarmaya UĞRAMAZ: kurtarma denemesi harcamak
                # aracı saniyelerce duraklatıyordu.
                'e_stop':           'MISSION_ABORT',
                'rampa_waypoint':   'RAMPA',
            }
        )

        smach.StateMachine.add(
            'RAMPA',
            RampaState(node, det_store, saat, rampa_tirmanis, rampa_inis),
            transitions={
                'completed': 'NAVIGATE',
                'failed':    'ERROR_RECOVERY',
                'e_stop':    'MISSION_ABORT',
            }
        )

        smach.StateMachine.add(
            'HIZLANMA',
            HizlanmaState(node, det_store, saat),
            transitions={
                'completed': 'NAVIGATE',
                'failed':    'ERROR_RECOVERY',
                'e_stop':    'MISSION_ABORT',
            }
        )

        smach.StateMachine.add(
            'SHOOT_APPROACH',
            ShootApproachState(node, det_store),
            transitions={
                'in_position': 'SHOOT',
                'failed':      'ERROR_RECOVERY',
                'e_stop':      'MISSION_ABORT',
            }
        )

        smach.StateMachine.add(
            'SHOOT',
            ShootState(node, det_store, saat),
            transitions={
                'shot_fired': 'NAVIGATE',
                'failed':     'ERROR_RECOVERY',
                'e_stop':     'MISSION_ABORT',
            }
        )

        smach.StateMachine.add(
            'MISSION_COMPLETE',
            MissionCompleteState(node, basarili=True),
            transitions={'done': 'GOREV_TAMAMLANDI'}
        )

        smach.StateMachine.add(
            'MISSION_ABORT',
            MissionCompleteState(node, basarili=False),
            transitions={'done': 'GOREV_IPTAL'}
        )

        smach.StateMachine.add(
            'ERROR_RECOVERY',
            ErrorRecoveryState(node),
            transitions={
                'recovered':     'NAVIGATE',
                # Aşamadan vazgeçildi — görev İPTAL DEĞİL, sıradakine devam.
                'sonraki_asama': 'NAVIGATE',
            }
        )

    # ROS2 spin ayrı thread'de (FSM bloklayıcı olduğu için)
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    node.get_logger().info('═══ LYDİA İKA Görev FSM v2.0 başlatıldı ═══')
    node.get_logger().info(f'Toplam waypoint : {len(waypoints)}')
    node.get_logger().info("'/mission_start' → True gönder → görev başlar")

    outcome = sm.execute()

    node.get_logger().info(f'FSM sonucu: {outcome}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
