from setuptools import find_packages, setup

package_name = 'offboard_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
            
        # Install 'simulation/' into 'share/offboard_control/simulation/'
        (f'share/{package_name}/simulation', 
         [f'{package_name}/simulation/processes.py']),
        
        # Install 'templates/' into 'share/offboard_control/templates/'
        (f'share/{package_name}/templates', 
         [f'{package_name}/templates/offboard_control_node.py']),

        (f'share/{package_name}/templates', 
         [f'{package_name}/templates/state_machine_template.py']),

        (f'share/{package_name}/templates', 
         [f'{package_name}/templates/star_movement.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UFRJ Harpia',
    maintainer_email='nautilus@poli.ufrj.br',
    description='Package to base the basic development',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # The basic example of how to use state machine tamplate,
            'star_movement=offboard_control.templates.star_movement:main'
        ],
    },
)
