#!/usr/bin/env python3
"""
TEKNOFEST 2026 IKA - Kayar Engel Hareket Kontrolcu
===================================================
Sartname 6.8:
  - Hiz: 20 cm/s
  - Parkur disina cikinca beklemeden ters yon
  - Surekli git-gel

Kullanim:
  python3 kayar_engel_hareket.py
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist
import math
import time


class KayarEngelKontrol(Node):
    def __init__(self):
        super().__init__('kayar_engel_kontrol')

        # =============================================
        # PARKUR AYARLARI - Gerekirse degistir
        # =============================================
        # Engelin dunya koordinatlarindaki baslangic konumu
        # (yarisma.world dosyasindaki pose ile ayni olmali)
        self.baslangic_x = -2.883
        self.baslangic_y =  9.969
        self.baslangic_z =  0.027

        # Engelin hareket ettigi eksen:
        #   'y' = Y ekseninde hareket (parkur genisligine dik)
        #   'x' = X ekseninde hareket
        self.hareket_ekseni = 'y'

        # Sartname degerleri
        self.HEDEF_HIZ  = 0.20   # 20 cm/s
        self.SOL_SINIR  = -1.5   # metre (baslangic konumundan)
        self.SAG_SINIR  =  1.5   # metre (baslangic konumundan)

        # =============================================
        # Durum degiskenleri
        # =============================================
        self.offset    = 0.0    # Baslangictan sapma (metre)
        self.yon       = 1.0    # +1: ileri, -1: geri
        self.dt        = 0.02   # 50 Hz guncelleme

        # =============================================
        # set_entity_state servisi
        # =============================================
        self.client = self.create_client(
            SetEntityState,
            '/set_entity_state'
        )

        self.get_logger().info('set_entity_state servisi bekleniyor...')
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Gazebo servisi bulunamadi, tekrar deneniyor...')

        self.get_logger().info(
            f'Kayar Engel basladi!\n'
            f'  Hiz: {self.HEDEF_HIZ} m/s\n'
            f'  Hareket ekseni: {self.hareket_ekseni.upper()}\n'
            f'  Sinirlar: [{self.SOL_SINIR}, {self.SAG_SINIR}] m'
        )

        # 50 Hz kontrol dongusu
        self.create_timer(self.dt, self.kontrol)

    def kontrol(self):
        # --- Yon kontrolu ---
        if self.offset >= self.SAG_SINIR:
            self.yon = -1.0
        elif self.offset <= self.SOL_SINIR:
            self.yon = 1.0

        # --- Pozisyonu guncelle ---
        self.offset += self.yon * self.HEDEF_HIZ * self.dt

        # --- Hedef konumu hesapla ---
        x = self.baslangic_x
        y = self.baslangic_y
        z = self.baslangic_z

        if self.hareket_ekseni == 'y':
            y += self.offset
        else:
            x += self.offset

        # --- Gazebo'ya gonder ---
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = 'kayar_engel'
        req.state.reference_frame = 'world'

        req.state.pose = Pose()
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z
        req.state.pose.orientation.w = 1.0

        req.state.twist = Twist()  # Sifir hiz

        self.client.call_async(req)


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
