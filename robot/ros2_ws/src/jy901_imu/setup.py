from setuptools import setup
package_name = 'jy901_imu'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kickpi',
    maintainer_email='kickpi@local',
    description='JY901 IMU driver (raw accel/gyro)',
    license='MIT',
    entry_points={'console_scripts': ['jy901_node = jy901_imu.jy901_node:main']},
)
