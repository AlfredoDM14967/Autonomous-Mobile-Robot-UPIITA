from setuptools import find_packages, setup

package_name = 'nodos_ros2'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['Led = nodos_ros2.Led:main',
                            'IHMremota = nodos_ros2.rover_IHMremota:main',
                            'motores = nodos_ros2.motores:main',
                            'scan_sub = nodos_ros2.scan_sub:main',
                            'imu_sub = nodos_ros2.imu_sub:main'
        ],
    },
)
