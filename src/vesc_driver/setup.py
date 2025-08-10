from setuptools import find_packages, setup

package_name = 'vesc_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/diff_drive_rover.launch.py',
            'launch/four_motor_rover.launch.py', 
            'launch/nav2_skid_steer.launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/motor_params.yaml',
            'config/four_motor_config.yaml',
            'config/nav2_skid_steer.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blake',
    maintainer_email='blake@todo.todo',
    description='ROS2 driver for VESC-compatible motor controllers with differential drive support',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vesc_motor_node = vesc_driver.vesc_motor_node:main',
            'vesc_diff_drive_node = vesc_driver.vesc_diff_drive_node:main',
            'vesc_skid_steer_node = vesc_driver.vesc_skid_steer_node:main',
        ],
    },
)
