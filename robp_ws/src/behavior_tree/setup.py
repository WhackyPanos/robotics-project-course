from setuptools import find_packages, setup

package_name = 'behavior_tree'

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
        'console_scripts': [
            'goCollect_bhv = behavior_tree.goCollect_bhv:main',
            'exploration_BT = behavior_tree.exploration_BT:main',
            'collection_BT = behavior_tree.collection_BT:main',
            'collection_BT_no_move = behavior_tree.collection_BT_no_move:main'
        ],
    },
)
