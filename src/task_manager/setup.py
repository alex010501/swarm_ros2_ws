from setuptools import find_packages, setup

package_name = 'task_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/task_manager_launch.py', 'launch/task_client_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='almaz',
    maintainer_email='alexmazurin0105@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_manager_node = task_manager.task_manager_node:main',
            'task_cli = task_manager.task_cli:main',
        ],
    },
)
