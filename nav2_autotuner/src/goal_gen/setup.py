from setuptools import find_packages, setup

package_name = 'goal_gen'

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
    maintainer='hariharan',
    maintainer_email='vshariharan2023@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'goal_gen = goal_gen.get_goal:main',
        'goal_nomans = goal_gen.no_mans_land_thing:main',
        'hard_coded = goal_gen.hard_coded:main',
        'nav2_switcher = goal_gen.nav2_switcher_params:main',
        'accumulator = goal_gen.accumulator_node:main'
        ],
    },
)
