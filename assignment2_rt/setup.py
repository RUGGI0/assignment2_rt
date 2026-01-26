from setuptools import setup

package_name = 'assignment2_rt'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='omegaruggio42@gmail.com',
    description='RT Assignment 2 (Python nodes)',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'teleop_node = assignment2_rt.teleop_node:main',
            'safety_node = assignment2_rt.safety_node:main',
        ],
    },
)
