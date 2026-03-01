from setuptools import setup

package_name = 'camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Drew Swinney',
    maintainer_email='drew@polyflowrobotics.com',
    description='Camera sensor node for Polyflow robots',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = camera.node:main',
        ],
    },
)
