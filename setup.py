from setuptools import find_packages, setup

package_name = 'robotx_pi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/robotx_pi/launch', ['launch/robotx_pi_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotx',
    maintainer_email='robotx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "rc_receiver = robotx_pi.rc_receiver:main",
            "working_mode = robotx_pi.working_mode_publisher:main",
            "remote_control = robotx_pi.remote_control:main",
            "servo_writer = robotx_pi.servo_writer:main",
            "led_control = robotx_pi.led_control:main",
            "angle_combiner = robotx_pi.angle_combiner:main",
            "latency_tester = robotx_pi.latency_timer:main",
            "throttle_power = robotx_pi.throttle_power:main",
        ],
    },
)
