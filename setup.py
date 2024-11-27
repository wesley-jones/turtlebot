from setuptools import find_packages, setup

package_name = 'turtlebot4_lava_integration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turtlebot4_lava_launch.py']),
    ],
    install_requires=['setuptools', 'lava-nc'],
    zip_safe=True,
    maintainer='Wesley H Jones',
    maintainer_email='wesoccer2003@yahoo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_to_lava = turtlebot4_lava_integration.lidar_to_lava:main',
            'lava_output_to_cmd = turtlebot4_lava_integration.lava_output_to_cmd:main'
        ],
    },
)
