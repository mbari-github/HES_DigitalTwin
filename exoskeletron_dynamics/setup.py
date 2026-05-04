from setuptools import find_packages, setup

package_name = 'exoskeletron_dynamics'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'exo_dynamics = exoskeletron_dynamics.new_dynamics_with_hand:main',
            'dynamics_stripped = exoskeletron_dynamics.dynamics_stripped:main',
        ],
    },
)
