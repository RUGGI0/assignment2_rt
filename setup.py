from setuptools import find_packages, setup

package_name = 'assignment2_rt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/ObstacleInfo.msg']),
        ('share/' + package_name + '/srv', ['srv/SetThreshold.srv', 'srv/GetVelAvg.srv',]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='omegaruggio42@gmail.com',
    description='RT Assignment 2 package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_node = assignment2_rt.teleop_node:main',
            'safety_node = assignment2_rt.safety_node:main',
        ],
    },
)

