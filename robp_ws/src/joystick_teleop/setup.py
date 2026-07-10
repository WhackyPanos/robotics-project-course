from setuptools import find_packages, setup

package_name = 'joystick_teleop'

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
    maintainer='group3-robot',
    maintainer_email='group3-robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['teleop = joystick_teleop.teleop:main',
                            'twist2duty = joystick_teleop.twist2duty:main' # added this (francisco)
        ],
    },
)
