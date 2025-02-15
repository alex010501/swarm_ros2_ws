from setuptools import find_packages, setup

package_name = 'youbot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/youbot_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='almaz',
    maintainer_email='alexmazurin0105@gmail.com',
    description='Strategic level controller of YouBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'youbot_control = youbot_control.youbot_control:main',
        ],
    },
)
