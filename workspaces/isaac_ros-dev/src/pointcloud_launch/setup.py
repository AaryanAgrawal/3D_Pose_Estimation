from setuptools import find_packages, setup

package_name = 'pointcloud_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/pointcloud_launch/pointcloud_launch', ['pointcloud_launch/pointcloud.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='f00567k',
    maintainer_email='f00567k@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
