import os
from glob import glob
from setuptools import setup

package_name = 'locomotion_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='max.ehrhardt@hotmail.de',
    description='Statemachine managing the activation of different locomotion modes.',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'locomotion_manager_node = locomotion_manager.locomotion_manager:main'
        ],
    },
)
