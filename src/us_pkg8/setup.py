from setuptools import find_packages, setup

package_name = 'us_pkg8'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='li',
    maintainer_email='zcl.iarc@gmail.com',
    description='Ultrasonic sensor node for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'us_receiver_node = us_pkg8.us_receiver_node:main',
            'us_show_node = us_pkg8.us_show_node:main',
        ],
    },
)
