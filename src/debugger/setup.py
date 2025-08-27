from setuptools import find_packages, setup

package_name = 'debugger'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/moveit_control_demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ymz',
    maintainer_email='yemingzhe3@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_simulator_node = debugger.camera_simulator_node:main',
            'target_pose_publisher = debugger.target_pose_publisher:main',
            'visualization_helper = debugger.visualization_helper:main',
        ],
    },
)
