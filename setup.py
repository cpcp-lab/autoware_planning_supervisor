from setuptools import find_packages, setup
from glob import glob

package_name = 'autoware_planning_supervisor'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daisuke Ishii',
    maintainer_email='dsksh@acm.org',
    description='A launcher for planning tasks supervised by a prescribed configuration.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planning_task = autoware_planning_supervisor.planning_task:main',
            'supervisor = autoware_planning_supervisor.supervisor:main',
        ],
    },
)
