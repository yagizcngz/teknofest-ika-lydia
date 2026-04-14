
import os
from glob import glob
from setuptools import setup

package_name = 'teknofest_ika'
parkur = 'src/ika_parkur'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'{parkur}/resource/teknofest_ika']),
        ('share/' + package_name, ['package.xml']),
        # Launch
        (os.path.join('share', package_name, 'launch'),
            glob(f'{parkur}/launch/*.launch.py')),
        # URDF
        (os.path.join('share', package_name, 'urdf'),
            glob(f'{parkur}/urdf/*.urdf')),
        # Meshes
        (os.path.join('share', package_name, 'meshes'),
            glob(f'{parkur}/meshes/*')),
        # Worlds
        (os.path.join('share', package_name, 'worlds'),
            glob(f'{parkur}/worlds/*.world')),
        # yarisma_parkuru modeli
        (os.path.join('share', package_name, 'models', 'yarisma_parkuru'),
            glob(f'{parkur}/models/yarisma_parkuru/*.*')),
        (os.path.join('share', package_name, 'models', 'yarisma_parkuru', 'materials', 'scripts'),
            glob(f'{parkur}/models/yarisma_parkuru/materials/scripts/*')),
        (os.path.join('share', package_name, 'models', 'yarisma_parkuru', 'materials', 'textures'),
            glob(f'{parkur}/models/yarisma_parkuru/materials/textures/*')),
        # kayar_engel modeli
        (os.path.join('share', package_name, 'models', 'kayar_engel'),
            glob(f'{parkur}/models/kayar_engel/*.*')),
        # tabela_13 modeli
        (os.path.join('share', package_name, 'models', 'tabela_13'),
            glob(f'{parkur}/models/tabela_13/*.*')),
        (os.path.join('share', package_name, 'models', 'tabela_13', 'materials', 'scripts'),
            glob(f'{parkur}/models/tabela_13/materials/scripts/*')),
        (os.path.join('share', package_name, 'models', 'tabela_13', 'materials', 'textures'),
            glob(f'{parkur}/models/tabela_13/materials/textures/*')),
        # tabela_5 modeli
        (os.path.join('share', package_name, 'models', 'tabela_5'),
            glob(f'{parkur}/models/tabela_5/*.*')),
        (os.path.join('share', package_name, 'models', 'tabela_5', 'materials', 'scripts'),
            glob(f'{parkur}/models/tabela_5/materials/scripts/*')),
        (os.path.join('share', package_name, 'models', 'tabela_5', 'materials', 'textures'),
            glob(f'{parkur}/models/tabela_5/materials/textures/*')),
        # dik_engel modeli
        (os.path.join('share', package_name, 'models', 'dik_engel'),
            glob(f'{parkur}/models/dik_engel/*.*')),
        # engebeli_arazi modeli
        (os.path.join('share', package_name, 'models', 'engebeli_arazi'),
            glob(f'{parkur}/models/engebeli_arazi/*.*')),
        # kayar_engel_model
        (os.path.join('share', package_name, 'models', 'kayar_engel_model'),
            glob(f'{parkur}/models/kayar_engel_model/*.*')),
        # materials
        (os.path.join('share', package_name, 'materials', 'scripts'),
            glob(f'{parkur}/materials/scripts/*')),
        (os.path.join('share', package_name, 'materials', 'textures'),
            glob(f'{parkur}/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Teknofest IKA Paketi',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'baslat = teknofest_ika.robot_beyni:main',
        ],
    },
)
