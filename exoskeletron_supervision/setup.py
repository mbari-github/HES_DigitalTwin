from setuptools import find_packages, setup

package_name = 'exoskeletron_supervision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/test',
            ['test/test_exo_bridge_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mbari',
    maintainer_email='mbari@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
            'launch_testing',
        ],
    },
    entry_points={
        'console_scripts': [
            'exo_bridge = exoskeletron_supervision.exo_bridge:main',
        ],
    },
)