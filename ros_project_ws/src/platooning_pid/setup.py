from setuptools import find_packages, setup

package_name = 'platooning_pid'

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
    maintainer='buvindu-suraweera',
    maintainer_email='buvindudsuraweera@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pid_node = platooning_pid.pid_node:main',
            'mock_leader = platooning_pid.mock_leader:main'
        ],
    },
)
