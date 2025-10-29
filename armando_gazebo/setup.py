import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'armando_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Indice del pacchetto
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # File package.xml
        ('share/' + package_name, ['package.xml']),

        # Cartella launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Cartella URDF/XACRO
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),

        # Cartella config (se serve)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Gazebo simulation package for Armando robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
