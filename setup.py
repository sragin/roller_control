from setuptools import setup

package_name = 'roller_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jongpil Kim',
    maintainer_email='jpkim@koceti.re.kr',
    description='Autonomous Vibration Roller Controller',
    license='koceti',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_key = roller_control.teleop_key:main',
            'roller_publisher = roller_control.roller_publisher:main',
            'roller_controller = roller_control.roller_controller:main',
            'base_controller = roller_control.base_controller:main',
            'roller_gui = roller_control.roller_control_ui:main',
        ],
    },
)
