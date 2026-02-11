import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'dual_robot_nav'

setup(
    name=package_name,  # Make sure this matches!
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
        # Include all world files
        (os.path.join('share', package_name, 'worlds'), 
            glob(os.path.join('worlds', '*.world'))),
        # Include maps if they exist
        (os.path.join('share', package_name, 'maps'), 
            glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anaskh007',
    maintainer_email='anaskh007@todo.todo',
    description='Dual Robot FMS Navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fms_smart_nav = dual_robot_nav.fms_smart_nav:main',
            'single_nav_goal = dual_robot_nav.single_nav_goal:main',
            
        ],
    },
)
