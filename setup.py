from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tourobo_2026_auto_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "map"), glob("map/*")),
        (os.path.join("share", package_name, "path"), glob("path/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aratahorie',
    maintainer_email='aratahorie89@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest',],
    },
    entry_points={
        'console_scripts': [
            "subscribe_twist_node = tourobo_2026_auto_robot.subscribe_twist_node:main",
            "joy2twist_node = tourobo_2026_auto_robot.joy2twist_node:main",
            "publish_odometry_node = tourobo_2026_auto_robot.publish_odometry_node:main",
        ],
    },
)
