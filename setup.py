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
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*') if os.path.exists('maps') else []),
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*') if os.path.exists('meshes') else []),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*') if os.path.exists('worlds') else []),
        (os.path.join('share', package_name, 'models'),
            glob('models/**/*.pt', recursive=True) + glob('models/**/*.engine', recursive=True)
            if os.path.exists('models') else []),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        'numpy',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='Magnesia',
    description='LYDIA IKA',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ── Gömülü ─────────────────────────────────────────────────
            'seri_kopru           = teknofest_ika.gomulu.seri_kopru:main',
            'taret_rc_koprusu     = teknofest_ika.gomulu.taret_rc_koprusu:main',
            # ── Otonomi ────────────────────────────────────────────────
            'misyon_fsm           = teknofest_ika.otonomi.misyon_fsm:main',
            'terrain_adapter      = teknofest_ika.otonomi.terrain_adapter:main',
            'veri_paketi          = teknofest_ika.otonomi.veri_paketi:main',
            'ackermann_converter  = teknofest_ika.otonomi.ackermann_converter:main',
            'anti_rollback        = teknofest_ika.otonomi.anti_rollback:main',
            'imu_guvenlik         = teknofest_ika.otonomi.imu_guvenlik:main',
            'mod_yoneticisi       = teknofest_ika.otonomi.mod_yoneticisi:main',
            'e_stop_node          = teknofest_ika.otonomi.e_stop_node:main',
            'watchdog             = teknofest_ika.otonomi.watchdog:main',
            # ── Görsel ─────────────────────────────────────────────────
            'kayar_engel_kalman   = teknofest_ika.gorsel.kayar_engel_kalman:main',
            'kayar_engel_costmap  = teknofest_ika.gorsel.kayar_engel_costmap:main',
            'koni_costmap         = teknofest_ika.gorsel.koni_costmap:main',
            'preprocessing_node   = teknofest_ika.gorsel.preprocessing_node:main',
            'yolo_detection_node  = teknofest_ika.gorsel.yolo_detection_node:main',
            'cone_fusion_node     = teknofest_ika.gorsel.cone_fusion_node:main',
            'lane_detection_node  = teknofest_ika.gorsel.lane_detection_node:main',
            'targeting_node       = teknofest_ika.gorsel.targeting_node:main',
            'servo_controller_node = teknofest_ika.gorsel.servo_controller_node:main',
            'yolo_adapter_node    = teknofest_ika.gorsel.yolo_adapter_node:main',
            'scan_relay           = teknofest_ika.gorsel.scan_relay:main',
            'map_image_node       = teknofest_ika.gorsel.map_image_node:main',
        ],
    },
)
