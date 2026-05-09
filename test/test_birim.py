# Copyright 2025 Magnesia Takimi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Kritik fonksiyon birim testleri -- donanim gerektirmez.

Kullanim:
    python3 test/test_birim.py
    pytest test/test_birim.py -v
"""
import math
import struct


# --- 1. Ackermann kinematik ---
L = 0.55
DELTA_MAX = 0.5236  # 30deg


def ackermann(v, w):
    if abs(v) < 1e-4:
        return math.copysign(DELTA_MAX, w) if abs(w) > 1e-4 else 0.0
    return max(-DELTA_MAX, min(DELTA_MAX, math.atan(L * w / v)))


def test_ackermann():
    assert math.isclose(math.degrees(ackermann(1.0, 0.0)), 0.0, abs_tol=0.01)
    assert ackermann(1.0, 0.5) > 0
    assert ackermann(1.0, -0.5) < 0
    assert math.isclose(ackermann(0.0, 1.0), DELTA_MAX, abs_tol=0.01)
    assert math.isclose(ackermann(0.0, -1.0), -DELTA_MAX, abs_tol=0.01)
    assert math.isclose(ackermann(0.0, 0.0), 0.0, abs_tol=0.01)
    assert math.isclose(ackermann(0.1, 10.0), DELTA_MAX, abs_tol=0.01)
    assert math.isclose(math.degrees(math.atan(L * 0.0 / -1.0)), 0.0, abs_tol=0.01)


# --- 2. Enkoder overflow korumasi ---
def delta(yeni, eski, maks=1024):
    d = yeni - eski
    yarim = maks // 2
    if d > yarim:
        d -= maks
    if d < -yarim:
        d += maks
    return d


def test_encoder_overflow():
    assert delta(10, 5) == 5
    assert delta(5, 10) == -5
    assert delta(1, 1023) == 2
    assert delta(1023, 1) == -2
    assert delta(512, 0) == 512
    assert delta(0, 0) == 0


# --- 3. Binary paket CRC ---
def make_pkt(cmd, v0, v1):
    v0 = max(-32768, min(32767, v0))
    v1 = max(-32768, min(32767, v1))
    veri = struct.pack('>hh', v0, v1)
    crc = cmd ^ veri[0] ^ veri[1] ^ veri[2] ^ veri[3]
    return struct.pack('BB4sBB', 0xAA, cmd, veri, crc, 0x55)


def verify(pkt):
    if pkt[0] != 0xAA or pkt[7] != 0x55:
        return False
    crc = pkt[1] ^ pkt[2] ^ pkt[3] ^ pkt[4] ^ pkt[5]
    return pkt[6] == crc


def test_binary_crc():
    for cmd, v0, v1 in [
        (0x01, 1000, 1500),
        (0x02, 0, 0),
        (0x03, 1, 0),
        (0x03, 0, 0),
        (0x04, 0, 0),
        (0x01, -1000, 3000),
    ]:
        pkt = make_pkt(cmd, v0, v1)
        assert verify(pkt) and len(pkt) == 8


# --- 4. Batarya yuzde ---
V_MIN, V_MAX = 14.0, 16.8


def bat_pct(v):
    return round(max(0.0, min(1.0, (v - V_MIN) / (V_MAX - V_MIN))) * 100)


def test_battery_percentage():
    assert bat_pct(16.8) == 100
    assert bat_pct(14.0) == 0
    assert bat_pct(15.4) == 50
    assert bat_pct(17.0) == 100
    assert bat_pct(13.0) == 0
    assert bat_pct(15.8) == 64


# --- 5. IMU Quaternion -> Roll / Pitch ---
def quat2rp(qw, qx, qy, qz):
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.degrees(math.atan2(sinr, cosr))
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.degrees(math.asin(sinp))
    return roll, pitch


def test_quaternion_to_rpy():
    r, p = quat2rp(1, 0, 0, 0)
    assert math.isclose(r, 0.0, abs_tol=0.01)
    assert math.isclose(p, 0.0, abs_tol=0.01)

    a = math.radians(15)
    r, p = quat2rp(math.cos(a / 2), math.sin(a / 2), 0, 0)
    assert math.isclose(r, 15.0, abs_tol=0.01)
    assert math.isclose(p, 0.0, abs_tol=0.01)

    a = math.radians(10)
    r, p = quat2rp(math.cos(a / 2), 0, math.sin(a / 2), 0)
    assert math.isclose(r, 0.0, abs_tol=0.01)
    assert math.isclose(p, 10.0, abs_tol=0.01)

    a = math.radians(20)
    r, p = quat2rp(math.cos(a / 2), math.sin(a / 2), 0, 0)
    assert abs(r) >= 19.99


# --- 6. Anti-rollback kosul mantigi ---
PITCH_TH = math.radians(10.0)
VEL_TH = 0.05


def rollback(pitch_deg, vel):
    pitch = math.radians(pitch_deg)
    return pitch > PITCH_TH and vel < -VEL_TH


def test_anti_rollback():
    assert rollback(12.0, -0.1) is True
    assert rollback(12.0, 0.5) is False
    assert rollback(2.0, -0.1) is False
    assert rollback(9.9, -0.2) is False
    assert rollback(11.0, 0.0) is False
    assert rollback(20.0, -0.5) is True


# --- 7. PID Integral Windup Korumasi ---
def test_pid_windup():
    INTEGRAL_MAX = 30.0
    integral = 0.0
    for _ in range(400):
        hata = 100.0
        dt = 0.05
        integral = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, integral + hata * dt))
    assert math.isclose(integral, INTEGRAL_MAX, abs_tol=0.01)
    assert integral <= INTEGRAL_MAX

    integral2 = 0.0
    for _ in range(400):
        integral2 = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, integral2 + (-100.0) * 0.05))
    assert math.isclose(integral2, -INTEGRAL_MAX, abs_tol=0.01)


# --- 8. Odometri dt Guard ---
def dt_gecerli(dt):
    return 0.0 < dt < 1.0


def test_odometry_dt_guard():
    assert dt_gecerli(0.02) is True
    assert dt_gecerli(0.5) is True
    assert dt_gecerli(1.5) is False
    assert dt_gecerli(-0.1) is False
    assert dt_gecerli(0.0) is False
    assert dt_gecerli(0.999) is True
    assert dt_gecerli(1.0) is False


# --- 9. IMU Guvenlik Hiz Kisit Mantigi ---
WARN = 8.0
STOP = 15.0
ESTOP = 20.0
NMAX = 2.0
FREN = 0.4
BAT_HIZ = 1.0


def guvenlik_hiz(roll_deg, pitch_deg, bat_pct_val):
    roll_abs = abs(roll_deg)
    if roll_abs >= ESTOP:
        hiz = 0.0
    elif roll_abs >= STOP:
        hiz = 0.0
    elif roll_abs >= WARN:
        oran = 1.0 - (roll_abs - WARN) / (STOP - WARN)
        hiz = NMAX * 0.5 * max(0.0, oran)
    elif pitch_deg <= -15.0:
        hiz = FREN
    else:
        hiz = NMAX

    if bat_pct_val < 10:
        hiz = min(hiz, 0.0)
    elif bat_pct_val < 30:
        hiz = min(hiz, BAT_HIZ)
    return hiz


def test_imu_safety_speed():
    assert math.isclose(guvenlik_hiz(0, 0, 100), 2.0, abs_tol=0.01)
    assert guvenlik_hiz(8, 0, 100) < 2.0
    assert math.isclose(guvenlik_hiz(15, 0, 100), 0.0, abs_tol=0.01)
    assert math.isclose(guvenlik_hiz(21, 0, 100), 0.0, abs_tol=0.01)
    assert math.isclose(guvenlik_hiz(0, -16, 100), FREN, abs_tol=0.01)
    assert math.isclose(guvenlik_hiz(0, 0, 20), BAT_HIZ, abs_tol=0.01)
    assert math.isclose(guvenlik_hiz(0, 0, 5), 0.0, abs_tol=0.01)
    assert guvenlik_hiz(5, 0, 20) <= BAT_HIZ


# Script olarak calistirildiginda da calissin
if __name__ == '__main__':
    import sys

    FAIL = 0
    PASS = 0

    def _run(fn):
        global FAIL, PASS
        try:
            fn()
            PASS += 1
            print(f"  [OK] {fn.__name__}")
        except AssertionError as e:
            FAIL += 1
            print(f"  [FAIL] {fn.__name__}: {e}")

    print("=== Birim Testleri ===")
    _run(test_ackermann)
    _run(test_encoder_overflow)
    _run(test_binary_crc)
    _run(test_battery_percentage)
    _run(test_quaternion_to_rpy)
    _run(test_anti_rollback)
    _run(test_pid_windup)
    _run(test_odometry_dt_guard)
    _run(test_imu_safety_speed)

    print(f"\n{'='*45}")
    print(f"  TOPLAM: {PASS+FAIL} test | {PASS} GECTI | {FAIL} BASARISIZ")
    print(f"{'='*45}")
    sys.exit(0 if FAIL == 0 else 1)
