#!/usr/bin/env python3
"""
anti_rollback.py - Yokus Kalkis ve ESP Destek Unitesi (Anti-Rollback 2.0)

Dik egimde aracin geri veya ileri (yokus asagi) kontrolsuz kaymasini engeller.
Tespit: Odom (tekerlek) yerine IMU ivmesi (linear_acceleration.x) kullanilir.
Mantigi:
  1. Kayma aninda mekanik fren yapar (Durum 1).
  2. Fren yetmezse dinamik gaz basar ve direksiyonu yumusatir (Durum 2).
  3. Maksimum sinira ragmen kayiyorsa E-STOP basar (Durum 3).
  4. Toparladiginda ekstra gucu bir sure daha destek olarak surdurur.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time
import math

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt16

from teknofest_ika.otonomi.topics import (
    IMU_TOPIC, MUX_CMD_VEL_TOPIC, ANTI_ROLLBACK_CMD_TOPIC, ANTI_ROLLBACK_AKTIF_TOPIC,
    E_STOP_TOPIC, E_STOP_FORCE_IMU_TOPIC, FREN_KOMUT_TOPIC, IMU_PITCH_RAMP_THRESHOLD
)
from teknofest_ika.otonomi.pure_logic import quat_to_roll_pitch_deg

# Eşikler
RAMP_PITCH_THRESHOLD_DEG = IMU_PITCH_RAMP_THRESHOLD
ACCEL_SLIP_THRESHOLD     = -0.1   # m/s^2 (X ekseninde geriye dogru fiziksel ivmelenme)
NOISE_FILTER_DURATION    = 0.2    # s (Yanlis alarmlari engellemek icin)
BRAKE_WAIT_DURATION      = 0.5    # s (Fren disklerinin mekanik kapanma suresi)
MAX_RECOVERY_SPEED       = 2.0    # m/s (Motor zorlama siniri)
THROTTLE_INCREMENT       = 0.1    # m/s (Artirma adimi)
THROTTLE_RAMP_DELAY      = 0.5    # s (Kademeli artis bekleme suresi)
BOOST_DECAY_TIME         = 2.0    # s (Yokus bitince gucun sifirlanma suresi)

KONTROL_HZ = 20.0

# State Machine Durumlari
STATE_IDLE             = 0
STATE_BRAKE_HOLD       = 1
STATE_DYNAMIC_THROTTLE = 2
STATE_ESTOP            = 3
STATE_FUSION           = 4


class AntiRollback(Node):
    def __init__(self):
        super().__init__('anti_rollback')

        self._state = STATE_IDLE
        self._pitch = 0.0
        self._accel_x = 0.0
        self._e_stop = False

        self._yolo_cmd = Twist()
        
        self._slip_start_time = 0.0
        self._state_start_time = 0.0
        self._last_ramp_time = 0.0
        self._hill_crest_time = 0.0
        
        self._boost = 0.0
        
        qos_be  = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Abonelikler
        self.create_subscription(Imu, IMU_TOPIC, self._imu_cb, qos_be)
        self.create_subscription(Twist, MUX_CMD_VEL_TOPIC, self._cmd_cb, 10)
        self.create_subscription(Bool, E_STOP_TOPIC, self._estop_cb, 10)

        # Yayinlar (Publishers)
        self._cmd_pub   = self.create_publisher(Twist, ANTI_ROLLBACK_CMD_TOPIC, qos_rel)
        self._durum_pub = self.create_publisher(Bool, ANTI_ROLLBACK_AKTIF_TOPIC, 10)
        self._fren_pub  = self.create_publisher(UInt16, FREN_KOMUT_TOPIC, 10)
        self._estop_pub = self.create_publisher(Bool, E_STOP_FORCE_IMU_TOPIC, 10)

        self.create_timer(1.0 / KONTROL_HZ, self._kontrol)
        self.get_logger().info('AntiRollback 2.0 (Yokus Kalkis ve ESP) Hazir.')

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        _, self._pitch = quat_to_roll_pitch_deg(q.w, q.x, q.y, q.z)
        self._accel_x = msg.linear_acceleration.x

    def _cmd_cb(self, msg: Twist):
        self._yolo_cmd = msg

    def _estop_cb(self, msg: Bool):
        self._e_stop = msg.data

    def _kontrol(self):
        if self._e_stop:
            self._state = STATE_IDLE
            self._durum_pub.publish(Bool(data=False))
            return

        now = time.time()

        # 1. Niyet ve Fiziksel Kayma Algilamasi (Simetrik Yokus Korumasi)
        yolo_linear = self._yolo_cmd.linear.x
        
        # Yokus Yukari Geri Kayma (YOLO ileri veya durmak istiyor ama ivme geri)
        is_slipping_back = (self._pitch > RAMP_PITCH_THRESHOLD_DEG and 
                            self._accel_x < ACCEL_SLIP_THRESHOLD and 
                            yolo_linear >= 0.0)
                            
        # Yokus Asagi Ileri Kayma (YOLO geri veya durmak istiyor ama ivme ileri)
        is_slipping_fwd = (self._pitch < -RAMP_PITCH_THRESHOLD_DEG and 
                           self._accel_x > abs(ACCEL_SLIP_THRESHOLD) and 
                           yolo_linear <= 0.0)
                           
        is_slipping = is_slipping_back or is_slipping_fwd

        # Gurultu Filtresi (Noise Filter)
        if is_slipping:
            if self._slip_start_time == 0.0:
                self._slip_start_time = now
        else:
            self._slip_start_time = 0.0
            
        slip_confirmed = (self._slip_start_time > 0 and (now - self._slip_start_time) >= NOISE_FILTER_DURATION)

        # 2. STATE MACHINE (Durum Makinesi) Yapisal Gecisler
        if self._state == STATE_IDLE:
            if slip_confirmed:
                self._state = STATE_BRAKE_HOLD
                self._state_start_time = now
                self.get_logger().warn(f"[Anti-Rollback] KAYMA TESPIT EDILDI (Pitch:{self._pitch:.1f}, Accel:{self._accel_x:.2f}) -> DURUM 1: MEKANIK FREN KILIDI")
                
        elif self._state == STATE_BRAKE_HOLD:
            if not is_slipping:
                # Sadece frenle durmayi basardik, yola devam edebilir
                self._state = STATE_IDLE
            elif (now - self._state_start_time) >= BRAKE_WAIT_DURATION:
                # Fren mekanizmasi 0.5 saniye calisti ama arac hala kayiyor (Camur/Buz)
                self._state = STATE_DYNAMIC_THROTTLE
                self._state_start_time = now
                self._last_ramp_time = now
                self._boost = 0.2  # Tork On Yukleme
                self.get_logger().warn("[Anti-Rollback] FREN YETMEDI! -> DURUM 2: DINAMIK GAZ VE ESP")
                
        elif self._state == STATE_DYNAMIC_THROTTLE:
            if not is_slipping:
                # Motor gucuyle kaymayi yendik, arac toparladi!
                self._state = STATE_FUSION
                self._state_start_time = now
                self.get_logger().info(f"[Anti-Rollback] KURTARILDI -> DURUM 4: YOKUS DESTEK MODU (Aktif Boost: +{self._boost:.1f} m/s)")
            else:
                # Arac hala kayiyor, gucu kademeli (PID) artir
                if (now - self._last_ramp_time) >= THROTTLE_RAMP_DELAY:
                    self._boost += THROTTLE_INCREMENT
                    self._last_ramp_time = now
                    if self._boost >= MAX_RECOVERY_SPEED:
                        self._state = STATE_ESTOP
                        self.get_logger().fatal("[Anti-Rollback] LIMIT ASILDI! FIZIKSEL SINIR! -> DURUM 3: GOREV IPTALI (E-STOP)")
                        
        elif self._state == STATE_ESTOP:
            # Sadece mesaj basmaya devam eder
            pass
            
        elif self._state == STATE_FUSION:
            # Yokus bittiginde (Eğim < 5) destek sonlandirma baslar
            if abs(self._pitch) < 5.0:
                if self._hill_crest_time == 0.0:
                    self._hill_crest_time = now
                
                # Boost degerini 2 saniyede eriyerek 0'a dusur (Bumpless Transfer Decay)
                gecen_sure = now - self._hill_crest_time
                if gecen_sure >= BOOST_DECAY_TIME or self._boost <= 0.0:
                    self._state = STATE_IDLE
                    self._hill_crest_time = 0.0
                    self._boost = 0.0
                    self.get_logger().info("[Anti-Rollback] YOKUS BITTI, GUC SIFIRLANDI -> DURUM 0: IDLE (KONTROL YOLO'DA)")
                else:
                    kalan_oran = 1.0 - (gecen_sure / BOOST_DECAY_TIME)
                    self._boost = self._boost * kalan_oran
            else:
                self._hill_crest_time = 0.0

        # 3. AKTARIM (Motorlara / Frenlere Emir Gonderimi)
        self._eylem_uygula()

    def _eylem_uygula(self):
        # Durum 0: Mudahale yok
        if self._state == STATE_IDLE:
            self._durum_pub.publish(Bool(data=False))
            return
            
        # Ackermann Converter'a bizden komut geldigini bildir (Override = True)
        self._durum_pub.publish(Bool(data=True))
        
        cmd = Twist()
        direction = 1.0 if self._pitch > 0 else -1.0
        
        if self._state == STATE_BRAKE_HOLD:
            # Sadece %100 Fren, Gaz = 0
            cmd.linear.x = 0.0
            cmd.angular.z = self._yolo_cmd.angular.z
            
        elif self._state == STATE_DYNAMIC_THROTTLE:
            # Frenleri birak (Hiz 0 olmadigi icin Ackermann freni kaldirir), Agresif Gaz Bas
            if self._yolo_cmd.linear.x == 0.0:
                cmd.linear.x = direction * self._boost
            else:
                if direction > 0:
                    cmd.linear.x = max(self._yolo_cmd.linear.x, 0.0) + self._boost
                else:
                    cmd.linear.x = min(self._yolo_cmd.linear.x, 0.0) - self._boost
                    
            # ESP (Yumusak Direksiyon) - Devrilme Onleme
            yaw = self._yolo_cmd.angular.z
            if yaw > 0.3: yaw = 0.3
            elif yaw < -0.3: yaw = -0.3
            cmd.angular.z = yaw
            
        elif self._state == STATE_ESTOP:
            # %100 Fren, Motor Sifir, Global E-STOP Alarm
            self._estop_pub.publish(Bool(data=True))
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
        elif self._state == STATE_FUSION:
            # Frenlere karisma, Gaz = YOLO + Hafizadaki Boost
            if self._yolo_cmd.linear.x == 0.0:
                # YOLO durmak istiyorsa (Yola insan atladiysa) asiri gucu KES (Hiz 0 oldugu icin Ackermann fren basar)
                cmd.linear.x = 0.0
                cmd.angular.z = self._yolo_cmd.angular.z
            else:
                # YOLO gitmek istiyorsa Boost ekle
                if direction > 0:
                    cmd.linear.x = self._yolo_cmd.linear.x + self._boost
                else:
                    cmd.linear.x = self._yolo_cmd.linear.x - self._boost
                cmd.angular.z = self._yolo_cmd.angular.z

        self._cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AntiRollback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
