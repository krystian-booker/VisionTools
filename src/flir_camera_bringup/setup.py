from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'flir_camera_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team2852',
    maintainer_email='team2852@todo.todo',
    description='Bringup for FLIR camera driver',
    license='Apache License 2.0',
    tests_require=['pytest'],
    data_files=[
        # register for ament
        ('share/ament_index/resource_index/packages',
            [f'resource/{package_name}']),
        # install package.xml
        (f'share/{package_name}', ['package.xml']),
        # install *all* your launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # (optional) if you have config/ presets:
        # (os.path.join('share', package_name, 'config'),
        #    glob('config/*.yaml')),
    ],
    entry_points={
        'console_scripts': [
            # none for now
        ],
        'launch': [
            # <cmd_name> = <python_module>:<factory_fn>
            'driver_node = flir_camera_bringup.launch.driver_node:generate_launch_description',
        ],
    },
)
