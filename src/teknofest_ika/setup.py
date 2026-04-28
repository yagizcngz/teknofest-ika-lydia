from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'teknofest_ika'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch dosyaları
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Config dosyaları
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # URDF
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        # Haritalar (varsa)
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*') if os.path.exists('maps') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Magnesia',
    maintainer_email='ozturkbugra684@gmail.com',
    description='LYDİA İKA — Teknofest Otomasyon Stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Ana seri köprü — Arduino <-> ROS2 binary protokol
            'seri_kopru = teknofest_ika.seri_kopru:main',
            # SMACH görev durum makinesi
            'misyon_fsm = teknofest_ika.misyon_fsm:main',
            # Tabela -> Nav2 parametre adaptörü
            'terrain_adapter = teknofest_ika.terrain_adapter:main',
            # Veri paketi kaydedici (Şartname 6.14)
            'veri_paketi = teknofest_ika.veri_paketi:main',
            # cmd_vel → AckermannDriveStamped dönüştürücü (bisiklet modeli)
            'ackermann_converter = teknofest_ika.ackermann_converter:main',
        ],
    },
)
