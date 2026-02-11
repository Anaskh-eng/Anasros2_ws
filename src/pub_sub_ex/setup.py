from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pub_sub_ex'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anouskh',
    maintainer_email='anouskh@example.com',
    description='Publisher Subscriber Example Package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pub = pub_sub_ex.pub_node:main',
            'sub = pub_sub_ex.sub_node:main',
            'nav_goals = pub_sub_ex.my_nav_script:main',
            'nav_goals_fixed = pub_sub_ex.nav_goals_fixed:main',
            'nav_simple = pub_sub_ex.nav_simple:main',  
        ],
    },
)