from setuptools import find_packages, setup

package_name = 'ohm_nav_tutorial'

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
    maintainer='chris',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = ohm_nav_tutorial.obstacle_avoidance:main', 
            'wall_follower = ohm_nav_tutorial.wall_follower:main',
        ],
    },
)
