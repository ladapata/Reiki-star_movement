from setuptools import find_packages, setup

package_name = 'movement_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [
            package_name + '/movement_controller.py',
            package_name + '/px4_controller.py',
            package_name + '/math_utils.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='harpia@labnet.nce.ufrj.br',
    description='Movement controller for PX4-Autopilot based drones',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
