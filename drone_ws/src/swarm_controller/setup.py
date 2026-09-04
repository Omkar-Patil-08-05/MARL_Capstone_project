from setuptools import find_packages, setup

package_name = 'swarm_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='capstone',
    maintainer_email='08omkarpatil05@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'two_drone_test = swarm_controller.two_drone_test:main',
            'smoke_test = swarm_controller.smoke_test:main',
            'deterministic_movement_test = swarm_controller.deterministic_movement_test:main',
            'two_drone_deterministic_test = swarm_controller.two_drone_deterministic_test:main',
            'qmix_drone_test = swarm_controller.qmix_drone_test:main',
            'swarm_runner = swarm_controller.swarm_runner:main',
            'yolo_human_detection = swarm_controller.yolo_human_detection:main',
            'swarm_perception_node = swarm_controller.swarm_perception_node:main'
        ],
    },
)
