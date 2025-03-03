from setuptools import find_packages, setup

package_name = 'rover_control'

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
    maintainer='mit',
    maintainer_email='mitanshc01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = rover_control.control_node:main',  #Entry point for rover_control node, if you delete this the code will show up "not executable"
            'sensor_dummy = rover_control.sensor_dummy:main', #Entry point for rover_control node
        ],
    },
)
