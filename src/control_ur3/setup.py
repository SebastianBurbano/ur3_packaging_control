from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control_ur3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sena',
    maintainer_email='sena@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'gui_control_ur3 = control_ur3.gui:main',
             'monitor_position = control_ur3.seguimiento:main',
             'jtc_client = control_ur3.empaque_4:main',
             'go_home = control_ur3.go_home:main',
             'iniciar = control_ur3.iniciar:main',
             'gripper_node = control_ur3.gripper_control:main',
             'captura_punto = control_ur3.captura_punto:main',
             'captura_varios = control_ur3.captura_varios:main',
             'empaque_prueba = control_ur3.empaque_prueba:main'
        ],
    },
)
