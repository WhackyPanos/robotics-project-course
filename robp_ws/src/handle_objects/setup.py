
from setuptools import find_packages, setup

package_name = 'handle_objects'
#packages=find_packages(include=['handle_objects', 'handle_objects.*']),
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyKDL'],
    zip_safe=True,
    maintainer='group3-robot',
    maintainer_email='domfri@kth.se',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pick_objects = handle_objects.pick_objects:main",
            "ik_solver = handle_objects.ik_solver:main"
        ],
    },
)

