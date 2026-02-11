from setuptools import find_packages, setup

package_name = 'mission_planner'

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
    maintainer='anaskh007',
    maintainer_email='anaskh007@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'single_robot_nav = mission_planner.single_robot_nav:main',
            'trial = mission_planner.trial:main',
            'single_robot_nav_fms2 = mission_planner.single_robot_nav_fms2:main',
            'dual_robot_commander_fms2 = mission_planner.dual_robot_commander_fms2:main',
            'single_robot_nav_fms3 = mission_planner.single_robot_nav_fms3:main',
            'single_robot_nav_fms4 = mission_planner.single_robot_nav_fms4:main',
            
        ],
    },
)
