from setuptools import setup

package_name = 'turtlebot3_pid_ajdust_comp'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jakub Czech',
    maintainer_email='kubaczech@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_publisher = turtlebot3_pid_ajdust_comp.vel_publisher:main',
        ],
    },
)
