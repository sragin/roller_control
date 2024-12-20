import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'roller_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']\
            + glob('resource/*.json')\
            + glob('resource/*.dbc')),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'python-statemachine',
        'cantools',
    ],
    zip_safe=True,
    maintainer='Jongpil Kim',
    maintainer_email='jpkim@koceti.re.kr',
    description='Autonomous Vibration Roller Controller',
    license='koceti',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator = roller_control.navigator:main',
            'roller_publisher = roller_control.roller_publisher:main',
            'roller_controller = roller_control.roller_controller:main',
            'base_controller = roller_control.base_controller:main',
            'roller_gui = roller_control.roller_control_ui:main',
            'teleop_joystick = roller_control.teleop_joy:main',
            'teleop2can = roller_control.teleop2can:main',
        ],
    },
)
