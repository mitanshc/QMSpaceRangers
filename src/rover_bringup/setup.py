from setuptools import setup
import os
from glob import glob

package_name = 'rover_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mit',
    maintainer_email='mitanshc01@gmail.com',
    description='Bringup package for the rover',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'qr_listener = rover_bringup.qr_listener:main',
        ],
    },
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
)
