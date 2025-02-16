from setuptools import find_packages, setup

package_name = 'handle_objects'

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
    maintainer='fm',
    maintainer_email='franmira2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "pick_objects = pick.pick_objects:main"
            #"lift_node = handle_objects.lift.lift_objects:main",  # Create `lift_objects.py`
            #"place_node = handle_objects.place.place_objects:main",  # Create `place_objects.py`
        ],
    },
)
