from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control_ur3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_dir={'': ''},  # <---- ESTE ES IMPORTANTE
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
             'gui_2_ur3 = control_ur3.gui_2:main',
             'jtc_client = control_ur3.empaque_4:main',
             'captura_varios = control_ur3.captura_varios:main',
             'order_monitor = control_ur3.order_monitor:main',
             'execute_trajectories = control_ur3.execute_trajectories:main',
             'opcion_1 = control_ur3.execute_trajectories_opcion_1:main',
             'definitivo = control_ur3.execute_trajectories_definitivo:main',
             'control_movimiento = control_ur3.control_movimiento:main',
        ],
    },
)

