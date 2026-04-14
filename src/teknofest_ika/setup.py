import os
from glob import glob
from setuptools import setup
package_name = 'teknofest_ika'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models', 'yarisma_parkuru'), glob('models/yarisma_parkuru/*.*')),
        (os.path.join('share', package_name, 'models', 'yarisma_parkuru', 'materials', 'scripts'), glob('models/yarisma_parkuru/materials/scripts/*')),
        (os.path.join('share', package_name, 'models', 'yarisma_parkuru', 'materials', 'textures'), glob('models/yarisma_parkuru/materials/textures/*')),
        (os.path.join('share', package_name, 'materials', 'scripts'), glob('materials/scripts/*')),
        (os.path.join('share', package_name, 'materials', 'textures'), glob('materials/textures/*')),
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
