from setuptools import setup, find_packages

PACKAGE_NAME = 'ros2_twist_to_jetbot_motion'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/launch', ['launch/twist_to_motion_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ANI717',
    maintainer_email='animesh.ani@live.com',
    description='ROS2 Package to run Jetbot subscribed to Geometry TWIST message',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'execute = ros2_twist_to_jetbot_motion.twist_to_motion_function:main',
        ],
    },
)