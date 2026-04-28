/**
 * ika_iletisim_ornek.c — MEGA Bare-Metal Örnek Uygulama
 * =======================================================
 * Diyagrama göre MEGA'nın sorumlulukları:
 *   1. Jetson'dan binary paket al → VESC/EC808D/NANO'ya ilet
 *   2. MPU925 (I2C) → PKT_IMU_YP + PKT_IMU_R → Jetson
 *   3. AS5600 enkoder (ADC) → PKT_ENC → Jetson
 *   4. INA219 (I2C) → PKT_AKIM → Jetson
 *   5. Flysky iBUS → RC modu
 *   6. Watchdog: HB gelmezse PKT_DUR uygula
 *
 * MEGA → NANO alt protokolü:
 *   PKT_SERVO_PAN, PKT_SERVO_TLT, PKT_LAZER komutları
 *   MEGA tarafından aynen NANO'ya iletilir (aynı binary format).
 */

#include "ika_iletisim.h"

/* ─── Platform fonksiyon prototipleri ─────────────────────────── */
/* Bunları kendi register/HAL kodunuza göre doldurun              */

/* MEGA ↔ Jetson (UART0 veya UART1) */
extern void     jetson_uart_gonder(const uint8_t *veri, uint8_t boyut);

/* MEGA → NANO (UART2) */
extern void     nano_uart_gonder(const uint8_t *veri, uint8_t boyut);

/* VESC CAN-Bus */
extern void     vesc_hiz_ayarla(int16_t hiz_mms);

/* EC808D Pulse/Dir — direksiyon step motoru */
extern void     ec808d_aci_ayarla(int16_t yaw_cd);

/* Lazer relay (NANO üzerinden, ama MEGA flag tutar) */
/* Doğrudan MEGA relay varsa buraya ekle */

/* MPU925 — I2C */
extern int16_t  mpu925_yaw_dd(void);    /* 1/10 derece */
extern int16_t  mpu925_pitch_dd(void);
extern int16_t  mpu925_roll_dd(void);

/* AS5600 — ADC */
extern uint16_t as5600_sol_oku(void);   /* 0-1023 */
extern uint16_t as5600_sag_oku(void);

/* INA219 — I2C */
extern int16_t  ina219_akim_ma(void);   /* mA */
extern int16_t  ina219_voltaj_mv(void); /* mV */

/* Watchdog */
extern void     wdt_besle(void);
extern uint32_t tick_ms(void);          /* ms sayacı */


/* ─── Durum Değişkenleri ───────────────────────────────────────── */

static volatile uint8_t  _rx_buf[IKA_PKT_BOYUT];
static volatile uint8_t  _rx_idx   = 0;
static volatile bool     _rx_hazir = false;

/* Lazer aktifken motor komutlarını blokla — Şartname 6.10 */
static volatile bool     _lazer_aktif = false;

/* Watchdog sayacı */
static uint32_t _son_hb_ms = 0;
#define WDT_ESIK_MS  2000u   /* 2 saniye HB gelmezse → dur */


/* ─── UART RX Interrupt (Jetson → MEGA) ──────────────────────── */

void MEGA_Jetson_UART_IRQ(uint8_t byte)
{
    if (_rx_idx == 0 && byte != IKA_PKT_BASLA) {
        return;  /* Sync bekleniyor */
    }

    _rx_buf[_rx_idx++] = byte;

    if (_rx_idx >= IKA_PKT_BOYUT) {
        _rx_idx   = 0;
        _rx_hazir = true;
    }
}


/* ─── Gelen Paketi İşle ───────────────────────────────────────── */

static void paket_isle(const IKA_Paket *p)
{
    if (!ika_dogrula(p)) return;   /* CRC hata → at */

    int16_t v0 = ika_v0_oku(p);
    int16_t v1 = ika_v1_oku(p);

    switch ((IKA_Komut)p->komut) {

        case PKT_SURUCU:
            /**
             * v0 = hiz_mms  → VESC CAN
             * v1 = yaw_cd   → EC808D Pulse/Dir
             *
             * Şartname 6.10: Lazer aktifken hareket yok!
             */
            if (!_lazer_aktif) {
                vesc_hiz_ayarla(v0);
                ec808d_aci_ayarla(v1);
            }
            _son_hb_ms = tick_ms();   /* PKT_SURUCU da watchdog besler */
            break;

        case PKT_DUR:
            vesc_hiz_ayarla(0);
            ec808d_aci_ayarla(0);
            /* Lazer da kapat */
            _lazer_aktif = false;
            {
                IKA_Paket dur_pkt;
                ika_paket_olustur(&dur_pkt, PKT_LAZER, 0, 0);
                nano_uart_gonder((uint8_t *)&dur_pkt, IKA_PKT_BOYUT);
            }
            break;

        case PKT_LAZER:
            /**
             * Şartname 6.10 kuralları:
             *   - Lazer açılırken araç DURDURULMUŞ olmalı
             *   - Lazer aktifken motor komutu kabul edilmez
             *   - Lazer min 1 saniye hedefte kalır (NANO sayar)
             *
             * MEGA → motor blokla → NANO'ya ilet
             */
            _lazer_aktif = (v0 == 1);
            if (_lazer_aktif) {
                /* Güvenlik: motoru durdur */
                vesc_hiz_ayarla(0);
                ec808d_aci_ayarla(0);
            }
            /* Paketi NANO'ya ilet (aynı format) */
            nano_uart_gonder((const uint8_t *)p, IKA_PKT_BOYUT);
            break;

        case PKT_SERVO_PAN:
            /**
             * v0 = pan açısı [0-180°]
             * MEGA doğrudan NANO'ya iletir.
             * NANO Tower Pro MG958/MG996R servo PWM üretir.
             */
            {
                IKA_Paket pan_pkt;
                int16_t aci = ika_servo_kirp(v0);
                ika_paket_olustur(&pan_pkt, PKT_SERVO_PAN, aci, 0);
                nano_uart_gonder((uint8_t *)&pan_pkt, IKA_PKT_BOYUT);
            }
            break;

        case PKT_SERVO_TLT:
            /**
             * v0 = tilt açısı [0-180°]
             * MEGA doğrudan NANO'ya iletir.
             */
            {
                IKA_Paket tlt_pkt;
                int16_t aci = ika_servo_kirp(v0);
                ika_paket_olustur(&tlt_pkt, PKT_SERVO_TLT, aci, 0);
                nano_uart_gonder((uint8_t *)&tlt_pkt, IKA_PKT_BOYUT);
            }
            break;

        case PKT_HB:
            _son_hb_ms = tick_ms();
            break;

        default:
            break;
    }
}


/* ─── Periyodik Gönderimler ───────────────────────────────────── */

/** 100 Hz timer callback — enkoder */
void timer_enc_cb(void)
{
    IKA_Paket p;
    ika_paket_olustur(&p,
                      PKT_ENC,
                      (int16_t)as5600_sol_oku(),
                      (int16_t)as5600_sag_oku());
    jetson_uart_gonder((uint8_t *)&p, IKA_PKT_BOYUT);
}

/** 50 Hz timer callback — IMU (MPU925 I2C, MEGA'ya bağlı) */
void timer_imu_cb(void)
{
    IKA_Paket p;

    /* Paket 1: Yaw + Pitch */
    ika_paket_olustur(&p, PKT_IMU_YP,
                      mpu925_yaw_dd(),
                      mpu925_pitch_dd());
    jetson_uart_gonder((uint8_t *)&p, IKA_PKT_BOYUT);

    /* Paket 2: Roll */
    ika_paket_olustur(&p, PKT_IMU_R, mpu925_roll_dd(), 0);
    jetson_uart_gonder((uint8_t *)&p, IKA_PKT_BOYUT);
}

/** 10 Hz timer callback — akım ve voltaj izleme */
void timer_akim_cb(void)
{
    IKA_Paket p;
    ika_paket_olustur(&p,
                      PKT_AKIM,
                      ina219_akim_ma(),
                      ina219_voltaj_mv());
    jetson_uart_gonder((uint8_t *)&p, IKA_PKT_BOYUT);
}

/** 1 Hz timer callback — watchdog kontrolü */
void timer_wdt_cb(void)
{
    uint32_t gecen = tick_ms() - _son_hb_ms;
    if (gecen > WDT_ESIK_MS) {
        /* Heartbeat kesildi → acil dur */
        vesc_hiz_ayarla(0);
        ec808d_aci_ayarla(0);
        _lazer_aktif = false;

        IKA_Paket dur;
        ika_paket_olustur(&dur, PKT_LAZER, 0, 0);
        nano_uart_gonder((uint8_t *)&dur, IKA_PKT_BOYUT);
    }
}


/* ─── Ana Döngü ───────────────────────────────────────────────── */

void ika_ana_dongu(void)
{
    if (_rx_hazir) {
        _rx_hazir = false;
        paket_isle((const IKA_Paket *)_rx_buf);
    }
}


/* ─────────────────────────────────────────────────────────────────
 * NANO TARAFI — aynı ika_iletisim.h kullanılır
 *
 * NANO'nun sorumlulukları:
 *   - MEGA'dan PKT_SERVO_PAN / PKT_SERVO_TLT al → servo PWM
 *   - MEGA'dan PKT_LAZER al → relay / transistör tetikle
 *   - BMI160 (Gyroscope, I2C) → NANO'da var ama kullanılmıyor
 *     NOT: IMU için MPU925 (MEGA'da) tercih edilir!
 *     BMI160 sadece servo başlık açısı için yedek olarak tutulabilir.
 *
 * NANO Örnek (aynı interrupt pattern):
 *
 *   void NANO_UART_IRQ(uint8_t byte) { ... }  // aynı state machine
 *
 *   void nano_paket_isle(const IKA_Paket *p) {
 *       if (!ika_dogrula(p)) return;
 *       int16_t v0 = ika_v0_oku(p);
 *       switch (p->komut) {
 *           case PKT_SERVO_PAN:
 *               servo_pan_yaz(v0);   // analogWrite / timer PWM
 *               break;
 *           case PKT_SERVO_TLT:
 *               servo_tlt_yaz(v0);
 *               break;
 *           case PKT_LAZER:
 *               digitalWrite(LAZER_PIN, v0 ? HIGH : LOW);
 *               break;
 *           case PKT_DUR:
 *               servo_pan_yaz(IKA_SERVO_MERKEZ);
 *               servo_tlt_yaz(IKA_SERVO_MERKEZ);
 *               digitalWrite(LAZER_PIN, LOW);
 *               break;
 *       }
 *   }
 * ───────────────────────────────────────────────────────────────── */
