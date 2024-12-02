from setuptools import setup

package_name = 'R5A_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/ros2_control_config.yaml',
                                               'config/controllers.yaml',
                                               'config/robot_description.urdf']),
        ('share/' + package_name + '/launch', ['launch/hardware_interface.launch.py',
                                               'launch/controller_manager.launch.py',
                                               'launch/robot_state_publisher.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Hardware interface for R5A robotic arm.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slush_hardware_interface = R5A_hardware.slush_engine_hardware:main',
        ],
    },
)
