/**
 * ika_iletisim.h — LYDİA İKA Binary Haberleşme Protokolü
 * =========================================================
 * Versiyon : 2.0  (diyagram mimarisine göre güncellendi)
 * Ekip     : Otomasyon + Gömülü Sistemler ortak tanımı
 *
 * MİMARİ (diyagram):
 *   Jetson ──UART binary──► MEGA
 *   MEGA   ──CAN-Bus──────► VESC (ana tahrik motorları)
 *   MEGA   ──Pulse/Dir────► EC808D → NEMA23 (direksiyon)
 *   MEGA   ──UART──────────► NANO → Servo Pan/Tilt + Lazer
 *   MEGA   ──I2C──────────► MPU925 (IMU), INA219 (akım)
 *   UNO    ──UART──────────► MEGA (ACS712 akım)
 *
 * PAKET YAPISI (8 byte, sabit uzunluk):
 *
 *  Byte 0 : 0xAA          — Başlangıç (sync)
 *  Byte 1 : KOMUT         — Komut tipi (enum IKA_Komut)
 *  Byte 2 : VERI_0        — int16 #1 yüksek byte (MSB)
 *  Byte 3 : VERI_1        — int16 #1 düşük  byte (LSB)
 *  Byte 4 : VERI_2        — int16 #2 yüksek byte (MSB)
 *  Byte 5 : VERI_3        — int16 #2 düşük  byte (LSB)
 *  Byte 6 : CRC           — XOR(Byte1..Byte5)
 *  Byte 7 : 0x55          — Bitiş
 *
 * BYTE ORDER: Big-endian (MSB önce)
 *   int16 okuma: (VERI_0 << 8) | VERI_1
 *
 * SERI PORT (Jetson ↔ MEGA):
 *   Baud : 115200 — Çerçeve: 8N1 — Çift yönlü tam-duplex
 *
 * KOMUT TABLOSU:
 * ─────────────────────────────────────────────────────────────────────
 *  KOMUT         YÖN           INT16 #1              INT16 #2
 * ─────────────────────────────────────────────────────────────────────
 *  PKT_SURUCU    Jetson→MEGA   hiz_mms [mm/s]        yaw_cd [1/100°]
 *  PKT_DUR       Jetson→MEGA   0                      0
 *  PKT_LAZER     Jetson→MEGA   1=aç / 0=kapa          0
 *  PKT_SERVO_PAN Jetson→MEGA   pan_aci [0-180°]       0
 *  PKT_SERVO_TLT Jetson→MEGA   tilt_aci [0-180°]      0
 *  PKT_HB        Jetson→MEGA   0                      0
 * ─────────────────────────────────────────────────────────────────────
 *  PKT_ENC       MEGA→Jetson   sol_enc [0-1023]       sag_enc [0-1023]
 *  PKT_IMU_YP    MEGA→Jetson   yaw_dd  [1/10°]        pitch_dd [1/10°]
 *  PKT_IMU_R     MEGA→Jetson   roll_dd [1/10°]        0
 *  PKT_AKIM      MEGA→Jetson   motor_ma [mA]          batarya_mv [mV]
 * ─────────────────────────────────────────────────────────────────────
 *
 * SERVO AÇI KODLAMASI:
 *   pan_aci  : 0-180 derece, 90 = merkez (lazer düz ileri)
 *   tilt_aci : 0-180 derece, 90 = merkez (lazer yatay)
 *   Örnek: Pan 95° = sağa 5° kaydır → [0xAA][0x05][0x00][0x5F][0x00][0x00][CRC][0x55]
 *
 * ÖRNEK — Jetson 0.5 m/s ileri, 12.5° sağa direksiyon:
 *   [0xAA][0x01][0x01][0xF4][0x04][0xE2][CRC][0x55]
 *                      ←500→         ←1250→
 *
 * ÖRNEK — Pan servoyu 95° konuma getir (sağa 5°):
 *   [0xAA][0x05][0x00][0x5F][0x00][0x00][CRC][0x55]
 *                      ←95→
 *
 * WATCHDOG KURALI:
 *   PKT_HB veya PKT_SURUCU 2 saniyede bir gelmezse MEGA
 *   otomatik PKT_DUR uygular.
 *   Şartname 7.8: "haberleşme kesilirse araç acil durma moduna geçer"
 *
 * MEGA → NANO ALT PROTOKOLÜ:
 *   MEGA, Jetson'dan gelen PKT_SERVO_PAN / PKT_SERVO_TLT / PKT_LAZER
 *   komutlarını aynı binary formatta NANO'ya iletir.
 *   NANO bu paketleri parse edip servo PWM ve lazer relay'ı kontrol eder.
 */

#ifndef IKA_ILETISIM_H
#define IKA_ILETISIM_H

#include <stdint.h>
#include <stdbool.h>

/* ── Sabitler ─────────────────────────────────────────────────── */
#define IKA_PKT_BOYUT        8        /* Tüm paketler 8 byte sabittir */
#define IKA_PKT_BASLA        0xAAu    /* Başlangıç sync byte          */
#define IKA_PKT_BITIS        0x55u    /* Bitiş sync byte              */

#define IKA_SERVO_MERKEZ     90       /* Servo merkez açısı [derece]  */
#define IKA_SERVO_MIN        0        /* Minimum servo açısı          */
#define IKA_SERVO_MAX        180      /* Maksimum servo açısı         */

/* ── Komut Enum ───────────────────────────────────────────────── */
typedef enum {

    /* ── Jetson → MEGA ─────────────────────────────────────────── */

    PKT_SURUCU    = 0x01,
    /**< Ana hareket komutu.
     *   v0 = hiz_mms  : Hız [mm/s], negatif = geri
     *                   Aralık: -32768 .. +32767 (~32 m/s)
     *                   MEGA bu değeri VESC CAN'a iletir.
     *   v1 = yaw_cd   : Direksiyon [1/100 derece], negatif = sol
     *                   Aralık: -3000 .. +3000  (-30° .. +30°)
     *                   MEGA bu değeri EC808D Pulse/Dir'e çevirir. */

    PKT_DUR       = 0x02,
    /**< Acil dur. v0=v1=0.
     *   MEGA: VESC'e hız=0, EC808D'ye adım dur, NANO'ya dur.
     *   Lazer varsa kapat. */

    PKT_LAZER     = 0x03,
    /**< Lazer kontrolü.
     *   v0 = 1 → lazer aç, v0 = 0 → lazer kapa.
     *   Şartname 6.10: Lazer aktifken araç hareket etmeyecek.
     *   MEGA PKT_LAZER=1 aldığında motor komutlarını bloklar,
     *   paketi NANO'ya iletir. NANO relay'ı tetikler. */

    PKT_SERVO_PAN = 0x05,
    /**< Yatay servo (pan) açısı.
     *   v0 = pan_aci : 0-180 derece, 90 = merkez.
     *   MEGA paketi NANO'ya iletir. NANO servo PWM üretir.
     *   Nişan alma: hedef_hata_x > 0 → pan_aci artır (sağa). */

    PKT_SERVO_TLT = 0x06,
    /**< Dikey servo (tilt) açısı.
     *   v0 = tilt_aci : 0-180 derece, 90 = yatay.
     *   MEGA paketi NANO'ya iletir. NANO servo PWM üretir.
     *   Nişan alma: hedef_hata_y > 0 → tilt_aci azalt (aşağı). */

    PKT_HB        = 0x04,
    /**< Heartbeat. v0=v1=0. 400ms'de bir gönderilir.
     *   MEGA watchdog'u besler. NANO'ya iletmek şart değil. */

    /* ── MEGA → Jetson ─────────────────────────────────────────── */

    PKT_ENC       = 0x10,
    /**< Enkoder verisi.
     *   v0 = sol_enc : Sol enkoder [0-1023] (AS5600 10-bit ADC)
     *   v1 = sag_enc : Sağ enkoder [0-1023]
     *   100 Hz gönderilir. Jetson odometri hesaplar. */

    PKT_IMU_YP    = 0x11,
    /**< IMU — Yaw + Pitch (MPU925, I2C bağlı MEGA'ya).
     *   v0 = yaw_dd   : [1/10 derece] -1800..+1800
     *   v1 = pitch_dd : [1/10 derece] -900..+900
     *   50 Hz gönderilir. */

    PKT_IMU_R     = 0x12,
    /**< IMU — Roll (MPU925, aynı paket serisi).
     *   v0 = roll_dd  : [1/10 derece] -900..+900
     *   v1 = 0 (rezerve)
     *   50 Hz, PKT_IMU_YP hemen ardından gönderilir. */

    PKT_AKIM      = 0x13,
    /**< Akım ve voltaj izleme.
     *   v0 = motor_ma   : Motor akımı [mA] (INA219, I2C)
     *   v1 = batarya_mv : Batarya voltajı [mV] (INA219 veya ACS712)
     *   10 Hz gönderilir. Jetson düşük batarya uyarısı için kullanır. */

} IKA_Komut;

/* ── Paket Yapısı ─────────────────────────────────────────────── */
typedef struct __attribute__((packed)) {
    uint8_t  basla;     /* 0xAA                          */
    uint8_t  komut;     /* IKA_Komut                     */
    uint8_t  veri[4];   /* 2 × int16, big-endian         */
    uint8_t  crc;       /* XOR(komut ^ veri[0..3])       */
    uint8_t  bitis;     /* 0x55                          */
} IKA_Paket;

/* ── CRC ──────────────────────────────────────────────────────── */
static inline uint8_t ika_crc_hesapla(const IKA_Paket *p)
{
    return (uint8_t)(p->komut ^ p->veri[0] ^ p->veri[1]
                              ^ p->veri[2] ^ p->veri[3]);
}

/* ── Doğrulama ────────────────────────────────────────────────── */
static inline bool ika_dogrula(const IKA_Paket *p)
{
    return (p->basla == IKA_PKT_BASLA) &&
           (p->bitis == IKA_PKT_BITIS) &&
           (p->crc   == ika_crc_hesapla(p));
}

/* ── Paket Oluşturma ──────────────────────────────────────────── */
static inline void ika_paket_olustur(IKA_Paket *p,
                                     uint8_t    komut,
                                     int16_t    v0,
                                     int16_t    v1)
{
    p->basla   = IKA_PKT_BASLA;
    p->komut   = komut;
    p->veri[0] = (uint8_t)((uint16_t)v0 >> 8);
    p->veri[1] = (uint8_t)((uint16_t)v0 & 0xFF);
    p->veri[2] = (uint8_t)((uint16_t)v1 >> 8);
    p->veri[3] = (uint8_t)((uint16_t)v1 & 0xFF);
    p->crc     = ika_crc_hesapla(p);
    p->bitis   = IKA_PKT_BITIS;
}

/* ── int16 Okuma Yardımcıları ─────────────────────────────────── */
static inline int16_t ika_v0_oku(const IKA_Paket *p)
{
    return (int16_t)(((uint16_t)p->veri[0] << 8) | p->veri[1]);
}

static inline int16_t ika_v1_oku(const IKA_Paket *p)
{
    return (int16_t)(((uint16_t)p->veri[2] << 8) | p->veri[3]);
}

/* ── Servo Açı Sınır Kontrolü ─────────────────────────────────── */
static inline int16_t ika_servo_kirp(int16_t aci)
{
    if (aci < IKA_SERVO_MIN) return IKA_SERVO_MIN;
    if (aci > IKA_SERVO_MAX) return IKA_SERVO_MAX;
    return aci;
}

#endif /* IKA_ILETISIM_H */
