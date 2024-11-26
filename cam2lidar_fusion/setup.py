from setuptools import find_packages, setup

package_name = 'cam2lidar_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/launch', ['launch/fusion.launch.py']),
        ('share/' + package_name+'/launch', ['launch/fusion.launch_3d_lidar.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aslan',
    maintainer_email='mustafaslan137@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam2lidar_fusion = cam2lidar_fusion.fusion:main',
        ],
    },
)
