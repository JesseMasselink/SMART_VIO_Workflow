from setuptools import find_packages, setup

package_name = 'IMU_CAMERA_Hardware_Sync'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'IMU_CAMERA_Hardware_Sync': ['imu_data_pb2.py'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'protobuf'],
    zip_safe=True,
    maintainer='jesse',
    maintainer_email='jesse@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_serial_node = IMU_CAMERA_Hardware_Sync.imu_serial_node:main'
        ],
    },
)
