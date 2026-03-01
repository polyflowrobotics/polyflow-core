from setuptools import setup

package_name = 'mission_runner'

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
    description='Mission runner orchestrator node for Polyflow robots',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_runner_node = mission_runner.node:main',
        ],
    },
)
