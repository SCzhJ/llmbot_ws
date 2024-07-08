from setuptools import find_packages, setup

package_name = 'bt_check'

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
    maintainer='fyp',
    maintainer_email='shengcezhang02@gmail.com',
    description='this package checks whether the behavior tree is desired',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bt_serv = bt_check.bt_serv:main',
            'bt_client = bt_check.bt_client:main',
        ],
    },
)
