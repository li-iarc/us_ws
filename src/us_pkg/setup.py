from setuptools import setup

package_name = 'us_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'us_pkg.us_receiver_node'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
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
            'us_receiver_node = us_pkg.us_receiver_node:main',
            'us_drawline_node = us_pkg.us_drawline_node:main',
            'us_colorbox_node = us_pkg.us_colorbox_node:main',
        ],
    },
)
