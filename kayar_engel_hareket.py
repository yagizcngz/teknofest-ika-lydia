#!/usr/bin/env python3
"""
TEKNOFEST 2026 IKA - Kayar Engel Hareket Kontrolcu
===================================================
Gazebo'nun apply_joint_effort servisi ile engeli
20 cm/s hizla saga-sola hareket ettirir.

Derleme gerekmez. Dogrudan calistir:
  python3 kayar_engel_hareket.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import ApplyJointEffort
from builtin_interfaces.msg import Duration, Time
import math


class KayarEngelKontrol(Node):
    def __init__(self):
        super().__init__('kayar_engel_kontrol')

        # --- Sartname parametreleri ---
        self.HEDEF_HIZ   = 0.20   # 20 cm/s
        self.SOL_SINIR   = -1.5   # metre
        self.SAG_SINIR   =  1.5   # metre
        self.KP          = 80.0   # Hiz kontrol kazanci (kuvvet)

        # --- Durum ---
        self.mevcut_poz  = 0.0
        self.mevcut_hiz  = 0.0
        self.yon         = 1.0    # +1: saga, -1: sola

        # --- Joint state subscriber ---
        self.create_subscription(
            JointState,
            '/kayar_engel/joint_states',
            self.joint_cb,
            10
        )

        # --- apply_joint_effort servisi ---
        self.effort_client = self.create_client(
            ApplyJointEffort,
            '/apply_joint_effort'
        )

        # Servis hazir olana kadar bekle
        self.get_logger().info('apply_joint_effort servisi bekleniyor...')
        while not self.effort_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Gazebo servisi bulunamadi, tekrar deneniyor...')

        self.get_logger().info('Kayar Engel basladi! 20 cm/s git-gel hareketi aktif.')

        # 50 Hz kontrol dongusu
        self.create_timer(0.02, self.kontrol)

    def joint_cb(self, msg: JointState):
        if 'engel_slide' in msg.name:
            idx = msg.name.index('engel_slide')
            self.mevcut_poz = msg.position[idx]
            self.mevcut_hiz = msg.velocity[idx]

    def kontrol(self):
        # Sinira ulasinca beklemeden yon degistir
        if self.mevcut_poz >= self.SAG_SINIR - 0.05:
            self.yon = -1.0
        elif self.mevcut_poz <= self.SOL_SINIR + 0.05:
            self.yon = 1.0

        # Hedef hiz ile mevcut hiz arasindaki farki kapat
        hedef = self.yon * self.HEDEF_HIZ
        hata  = hedef - self.mevcut_hiz
        kuvvet = self.KP * hata

        # Kuvveti sinirla
        kuvvet = max(-150.0, min(150.0, kuvvet))

        # Gazebo'ya gonder
        req = ApplyJointEffort.Request()
        req.joint_name = 'kayar_engel::engel_slide'
        req.effort = kuvvet
        req.start_time = Time(sec=0, nanosec=0)
        req.duration = Duration(sec=0, nanosec=25_000_000)  # 25ms

        self.effort_client.call_async(req)


def main():
    rclpy.init()
    node = KayarEngelKontrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Durduruluyor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()