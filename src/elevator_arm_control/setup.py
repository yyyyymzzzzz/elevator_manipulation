from setuptools import find_packages, setup

package_name = 'elevator_arm_control'

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
    maintainer='ymz',
    maintainer_email='yemingzhe3@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller = elevator_arm_control.arm_controller:main',
            'arm_pose_calculator = elevator_arm_control.arm_pose_calculator:main',
            'trajectory_controller = elevator_arm_control.trajectory_controller:main',
        ],
    },
)
