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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Magnesia',
    description='LYDIA IKA',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'seri_kopru           = teknofest_ika.gomulu.seri_kopru:main',
            'misyon_fsm           = teknofest_ika.otonomi.misyon_fsm:main',
            'terrain_adapter      = teknofest_ika.otonomi.terrain_adapter:main',
            'veri_paketi          = teknofest_ika.otonomi.veri_paketi:main',
            'ackermann_converter  = teknofest_ika.otonomi.ackermann_converter:main',
            'anti_rollback        = teknofest_ika.otonomi.anti_rollback:main',
            'imu_guvenlik         = teknofest_ika.otonomi.imu_guvenlik:main',
            'kayar_engel_kalman   = teknofest_ika.gorsel.kayar_engel_kalman:main',
        ],
    },
)
