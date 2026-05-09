import os
from glob import glob
from setuptools import setup

package_name = 'teknofest_ika'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/teknofest_ika']),
        ('share/' + package_name, ['package.xml']),
        # Launch
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # URDF
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        # Meshes
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*')),
        # Worlds
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        # Maps
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        # yarisma_parkuru modeli
        (os.path.join('share', package_name, 'models', 'yarisma_parkuru'),
            glob('models/yarisma_parkuru/*.*')),
        (os.path.join('share', package_name, 'models', 'yarisma_parkuru', 'materials', 'scripts'),
            glob('models/yarisma_parkuru/materials/scripts/*')),
        (os.path.join('share', package_name, 'models', 'yarisma_parkuru', 'materials', 'textures'),
            glob('models/yarisma_parkuru/materials/textures/*')),
        # kayar_engel modeli
        (os.path.join('share', package_name, 'models', 'kayar_engel'),
            glob('models/kayar_engel/*.*')),
        # tabela_13 modeli
        (os.path.join('share', package_name, 'models', 'tabela_13'),
            glob('models/tabela_13/*.*')),
        (os.path.join('share', package_name, 'models', 'tabela_13', 'materials', 'scripts'),
            glob('models/tabela_13/materials/scripts/*')),
        (os.path.join('share', package_name, 'models', 'tabela_13', 'materials', 'textures'),
            glob('models/tabela_13/materials/textures/*')),
        # tabela_5 modeli
        (os.path.join('share', package_name, 'models', 'tabela_5'),
            glob('models/tabela_5/*.*')),
        (os.path.join('share', package_name, 'models', 'tabela_5', 'materials', 'scripts'),
            glob('models/tabela_5/materials/scripts/*')),
        (os.path.join('share', package_name, 'models', 'tabela_5', 'materials', 'textures'),
            glob('models/tabela_5/materials/textures/*')),
        # dik_engel modeli
        (os.path.join('share', package_name, 'models', 'dik_engel'),
            glob('models/dik_engel/*.*')),
        # engebeli_arazi modeli
        (os.path.join('share', package_name, 'models', 'engebeli_arazi'),
            glob('models/engebeli_arazi/*.*')),
        # kayar_engel_model
        (os.path.join('share', package_name, 'models', 'kayar_engel_model'),
            glob('models/kayar_engel_model/*.*')),
        # materials
        (os.path.join('share', package_name, 'materials', 'scripts'),
            glob('materials/scripts/*')),
        (os.path.join('share', package_name, 'materials', 'textures'),
            glob('materials/textures/*')),
        # config/ klasöründeki tüm YAML dosyalarını install'a kopyala
        # (ekf.yaml, nav2_params.yaml burada)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'pyserial>=3.5',
        'PyYAML>=5.4.0',
        'numpy>=1.21.0',
        'opencv-python>=4.5.0',
    ],
    zip_safe=True,
    maintainer='bugra16',
    maintainer_email='ozturkbugra684@gmail.com',
    description='Teknofest IKA Paketi',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'karar_nodu = teknofest_ika.otonomi.karar_nodu:main',
            'seri_kopru = teknofest_ika.gomulu.seri_kopru:main',
            'misyon_planlayici = teknofest_ika.otonomi.misyon_planlayici:main',
        ],
    },
)
