from setuptools import find_packages, setup

package_name = 'agv_controller'

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
    maintainer='nvidia',
    maintainer_email='yemingzhe@sjtu.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agv_target_controller = agv_controller.agv_target_controller:main',
            'system_monitor = agv_controller.system_monitor:main',
            'test_button_completion = agv_controller.test_button_completion:main',
            'test_agv_target_button = agv_controller.test_agv_target_button:main',
        ],
    },
)
