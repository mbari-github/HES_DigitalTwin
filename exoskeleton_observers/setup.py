from setuptools import find_packages, setup

package_name = 'exoskeleton_observers'

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
    maintainer='mbari',
    maintainer_email='mbari@todo.todo',
    description='Model-based observers for exoskeleton fault detection',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'observer_node = exoskeleton_observers.observer_node:main',
        ],
    },
)